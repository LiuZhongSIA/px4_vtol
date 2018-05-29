/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
// #include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>  //根据是不是vtol，决定姿态角速度期望的发送位置
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/v44_tilt_flag.h>
#include <uORB/topics/v44_control_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>

/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

// #define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	 0.3f
#define RATES_I_LIMIT_TS 0.5f //RATES_I_LIMIT_TS * max_tilt_angle
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
// #define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3

class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~MulticopterAttitudeControl();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_ctrl_state_sub;		/**< control state subscription */ //控制所需要的系统状态
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */ //姿态角速度期望
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */ //遥控器的输入信号
	int		_armed_sub;				/**< arming status subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */ //飞行器状态，包括了是不是处于vtol，是不是
	int 	_motor_limits_sub;		/**< motor limits subscription */
	int 	_battery_status_sub;	/**< battery status subscription */
	int     _v44_tilt_flag_sub;

	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_actuators_1_pub;
	orb_advert_t	_controller_status_pub;	/**< controller status publication */
	orb_advert_t    _v44_control_status_pub;

	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */     //根据是不是在vtol飞行器下，改变消息发布ID
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */ //抑制输出的断路器

	struct control_state_s				_ctrl_state;		/**< control state */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_controls_s			_actuators1;		
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
	struct mc_att_ctrl_status_s 		_controller_status; /**< controller status */
	struct battery_status_s				_battery_status;	/**< battery status */
	struct v44_tilt_flag_s              _v44_tilt_flag;
	struct v44_control_status_s         _v44_control_status;

	perf_counter_t	_loop_perf;			/**< loop performance counter */ //循环性能计数器
	perf_counter_t	_controller_latency_perf; //控制器潜在因素性能

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */ //误差积分
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */ //旋翼控制量
	math::Vector<3>		_rates_int_fw;	//FW控制积分器
	math::Vector<4>     _ts_1234;       //每个倾转舵机的平均倾转量
	math::Vector<3>     _att_control_ts;//tilt servo 的控制量

	math::Matrix<3, 3>  _I;				/**< identity matrix */ //单位矩阵

	enum vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
		FW_MODE,			    /**< vtol is in fixed wing mode */
		TRANSITION_BACK_P1,     /**< vtol is in back transition mode */ //Part 1
		TRANSITION_BACK_P2      //Part 2
	};

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t roll_rate_ff;                //前馈，提高跟踪性能
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t tpa_breakpoint;             //开始衰减滚转、俯仰角P的油门期望
		param_t tpa_slope;                  //开始衰减滚转、俯仰角P的油门期望速度
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;               //只用于outo模式，也是防止出现混控的饱和

		param_t acro_roll_max;              //acro模式下，杆量给的是姿态角速度
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t rattitude_thres;            //rattitude模式下，在一定范围内杆量给的是姿态角速度，超出后，给的是姿态角（滚转和俯仰）

		param_t vtol_type;                  //定义在vtol_att_control_params.c中
		param_t roll_tc;                    //时间常数，调节响应时间
		param_t pitch_tc;
		param_t vtol_opt_recovery_enabled;  //定义在vtol_att_control_params.c中，for tailsitters
		param_t vtol_wv_yaw_rate_scale;     //定义在vtol_att_control_params.c中

		param_t bat_scale_en;               //是否根据电池电池电量缩放输出

		// 新添加句柄
		param_t roll_rate_p_fw;
		param_t roll_rate_i_fw;
		param_t roll_rate_d_fw;
		param_t roll_rate_ff_fw;
		param_t pitch_rate_p_fw;
		param_t pitch_rate_i_fw;
		param_t pitch_rate_d_fw;
		param_t pitch_rate_ff_fw;
		param_t yaw_rate_p_fw;
		param_t yaw_rate_i_fw;
		param_t yaw_rate_d_fw;
		param_t yaw_rate_ff_fw;
		param_t vt_tilt_1_mc;
		param_t vt_tilt_1_fw;
		param_t vt_tilt_2_mc;
		param_t vt_tilt_2_fw;
		param_t vt_tilt_3_mc;
		param_t vt_tilt_3_fw;
		param_t vt_tilt_4_mc;
		param_t vt_tilt_4_fw;

	}		_params_handles;		        /**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;				/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float tpa_breakpoint;				/**< Throttle PID Attenuation breakpoint */
		float tpa_slope;					/**< Throttle PID Attenuation slope */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		float yaw_auto_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */
		math::Vector<3> auto_rate_max;		/**< attitude rate limits in auto modes */
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */
		float rattitude_thres;
		int vtol_type;						/**< 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe */
		bool vtol_opt_recovery_enabled;
		float vtol_wv_yaw_rate_scale;	    /**< Scale value [0, 1] for yaw rate setpoint  */

		int bat_scale_en;

		// 新添加的变量
		math::Vector<3> rate_p_fw;
		math::Vector<3> rate_i_fw;
		math::Vector<3> rate_d_fw;
		math::Vector<3>	rate_ff_fw;
		float vt_tilt_1_mc;
		float vt_tilt_1_fw;
		float vt_tilt_2_mc;
		float vt_tilt_2_fw;
		float vt_tilt_3_mc;
		float vt_tilt_3_fw;
		float vt_tilt_4_mc;
		float vt_tilt_4_fw;
	}		_params;

	TailsitterRecovery *_ts_opt_recovery;	/**< Computes optimal rates for tailsitter recovery */

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle motor limits status.
	 */
	void		vehicle_motor_limits_poll();

	/**
	 * Check for battery status updates.
	 */
	void		battery_status_poll();

	/**
	 * Check for tilt flag updates.
	 */
	void		v44_tilt_flag_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

// 命名空间，由于人类词汇量较少造成的
// 定义变量
namespace mc_att_control
{
	MulticopterAttitudeControl	*g_control;
}

// 构造函数
MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
	_control_task(-1),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_vehicle_status_sub(-1),
	_v44_tilt_flag_sub(-1),

	/* publications */
	_v_rates_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_1_pub(nullptr),
	_controller_status_pub(nullptr),
	_v44_control_status_pub(nullptr),
	_rates_sp_id(0),
	_actuators_id(0),

	_actuators_0_circuit_breaker_enabled(false),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")), //执行事件的消耗时间，产生一个新的计数器
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")), //如果计数器存在，这里仅仅获得其参考值；如果不存在，产生新的计数器
	_ts_opt_recovery(nullptr)

{
	// 初始化存储变量值
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_actuators1, 0, sizeof(_actuators1));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
	memset(&_motor_limits, 0, sizeof(_motor_limits));
	memset(&_controller_status, 0, sizeof(_controller_status));
	memset(&_v44_tilt_flag, 0, sizeof(_v44_tilt_flag));
	memset(&_v44_control_status, 0, sizeof(_v44_control_status));
	_vehicle_status.is_rotary_wing = true; //处于旋翼模式下

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.auto_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;
	_params.vtol_opt_recovery_enabled = false;
	_params.vtol_wv_yaw_rate_scale = 1.0f;
	_params.bat_scale_en = 0;
	_params.rate_p_fw.zero(); //新参数的初始化
	_params.rate_i_fw.zero();
	_params.rate_d_fw.zero();
	_params.rate_ff_fw.zero();
	_params.vt_tilt_1_mc = 0.0f;
	_params.vt_tilt_1_fw = 1.0f;
	_params.vt_tilt_2_mc = 0.0f;
	_params.vt_tilt_2_fw = 1.0f;
	_params.vt_tilt_3_mc = 0.0f;
	_params.vt_tilt_3_fw = 1.0f;
	_params.vt_tilt_4_mc = 0.0f;
	_params.vt_tilt_4_fw = 1.0f;

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();
	_rates_int_fw.zero();
	_ts_1234.zero();
	_att_control_ts.zero();

	_I.identity();

	// 参数句柄赋值
	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff	= 	param_find("MC_ROLLRATE_FF");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 	= 	param_find("MC_PITCHRATE_FF");
	_params_handles.tpa_breakpoint 	= 	param_find("MC_TPA_BREAK");
	_params_handles.tpa_slope	 	= 	param_find("MC_TPA_SLOPE");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.roll_rate_max	= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max	= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max	= 	param_find("MC_YAWRAUTO_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.rattitude_thres = 	param_find("MC_RATT_TH");
	_params_handles.vtol_type 		= 	param_find("VT_TYPE");
	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");
	_params_handles.vtol_opt_recovery_enabled	= param_find("VT_OPT_RECOV_EN");
	_params_handles.vtol_wv_yaw_rate_scale		= param_find("VT_WV_YAWR_SCL");
	_params_handles.bat_scale_en		= param_find("MC_BAT_SCALE_EN");
	// 新添加句柄
	_params_handles.roll_rate_p_fw	= 	param_find("MC_ROLLRA_P_FW");
	_params_handles.roll_rate_i_fw	= 	param_find("MC_ROLLRA_I_FW");
	_params_handles.roll_rate_d_fw	= 	param_find("MC_ROLLRA_D_FW");
	_params_handles.roll_rate_ff_fw= 	param_find("MC_ROLLRA_FF_FW");
	_params_handles.pitch_rate_p_fw	= 	param_find("MC_PITCHRA_P_FW");
	_params_handles.pitch_rate_i_fw	= 	param_find("MC_PITCHRA_I_FW");
	_params_handles.pitch_rate_d_fw	= 	param_find("MC_PITCHRA_D_FW");
	_params_handles.pitch_rate_ff_fw= 	param_find("MC_PITCHRA_FF_FW");
	_params_handles.yaw_rate_p_fw	= 	param_find("MC_YAWRA_P_FW");
	_params_handles.yaw_rate_i_fw	= 	param_find("MC_YAWRA_I_FW");
	_params_handles.yaw_rate_d_fw	= 	param_find("MC_YAWRA_D_FW");
	_params_handles.yaw_rate_ff_fw= 	param_find("MC_YAWRA_FF_FW");
	_params_handles.vt_tilt_1_mc    = 	param_find("VT_TILT_1_MC");
	_params_handles.vt_tilt_1_fw    = 	param_find("VT_TILT_1_FW");
	_params_handles.vt_tilt_2_mc    = 	param_find("VT_TILT_2_MC");
	_params_handles.vt_tilt_2_fw    = 	param_find("VT_TILT_2_FW");
	_params_handles.vt_tilt_3_mc    = 	param_find("VT_TILT_3_MC");
	_params_handles.vt_tilt_3_fw    = 	param_find("VT_TILT_3_FW");
	_params_handles.vt_tilt_4_mc    = 	param_find("VT_TILT_4_MC");
	_params_handles.vt_tilt_4_fw    = 	param_find("VT_TILT_4_FW");

	/* fetch initial parameter values */
	// 根据句柄，更新参数，这是初始化的参数
	parameters_update();

	// 仅仅立式起降飞机用到“最优恢复控制策略”
	if (_params.vtol_type == 0 && _params.vtol_opt_recovery_enabled) {
		// the vehicle is a tailsitter, use optimal recovery control strategy
		_ts_opt_recovery = new TailsitterRecovery();
	}
}

// 析构函数
MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	// 进入构造函数，说明需要跳出主任务
	if (_control_task != -1) { //会导致跳出task_main()中的大循环
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1); //直到_control_task == -1，跳出循环
	}

	if (_ts_opt_recovery != nullptr) {
		delete _ts_opt_recovery;
	}

	mc_att_control::g_control = nullptr;
}

// 地面站参数写入存储变量
int
MulticopterAttitudeControl::parameters_update()
{
	float v;
	float roll_tc, pitch_tc;

	param_get(_params_handles.roll_tc, &roll_tc);
	param_get(_params_handles.pitch_tc, &pitch_tc);

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc); //滚转和俯仰的时间常数，直接影响了内外环的P和内环的D
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc); //D减小，确实可以増快响应速度
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	param_get(_params_handles.tpa_breakpoint, &v);
	_params.tpa_breakpoint = v;
	param_get(_params_handles.tpa_slope, &v);
	_params.tpa_slope = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* auto angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.auto_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.auto_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_auto_max, &_params.yaw_auto_max);
	_params.auto_rate_max(2) = math::radians(_params.yaw_auto_max);

	/* manual rate control scale and auto mode roll/pitch rate limits */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	/* stick deflection needed in rattitude mode to control rates not angles */
	param_get(_params_handles.rattitude_thres, &_params.rattitude_thres);

	// vtol飞行器类型
	param_get(_params_handles.vtol_type, &_params.vtol_type);

	int tmp;
	param_get(_params_handles.vtol_opt_recovery_enabled, &tmp);
	_params.vtol_opt_recovery_enabled = (bool)tmp;

	param_get(_params_handles.vtol_wv_yaw_rate_scale, &_params.vtol_wv_yaw_rate_scale);

	param_get(_params_handles.bat_scale_en, &_params.bat_scale_en);

	// 在正常飞行中是用不到的，但是开发过程中可能用到
	// 是在模仿航空断路器实现子系统的禁能，使能的条件比较苛刻，使能后的结果就是不发布混控输入值
	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	// 更新新变量
	// roll
	param_get(_params_handles.roll_rate_p_fw, &v);
	_params.rate_p_fw(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_i_fw, &v);
	_params.rate_i_fw(0) = v;
	param_get(_params_handles.roll_rate_d_fw, &v);
	_params.rate_d_fw(0) = v * (ATTITUDE_TC_DEFAULT / roll_tc);
	param_get(_params_handles.roll_rate_ff_fw, &v);
	_params.rate_ff_fw(0) = v;
	// pitch
	param_get(_params_handles.pitch_rate_p_fw, &v);
	_params.rate_p_fw(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_i_fw, &v);
	_params.rate_i_fw(1) = v;
	param_get(_params_handles.pitch_rate_d_fw, &v);
	_params.rate_d_fw(1) = v * (ATTITUDE_TC_DEFAULT / pitch_tc);
	param_get(_params_handles.pitch_rate_ff_fw, &v);
	_params.rate_ff_fw(1) = v;
	// yaw
	param_get(_params_handles.yaw_rate_p_fw, &v);
	_params.rate_p_fw(2) = v;
	param_get(_params_handles.yaw_rate_i_fw, &v);
	_params.rate_i_fw(2) = v;
	param_get(_params_handles.yaw_rate_d_fw, &v);
	_params.rate_d_fw(2) = v;
	param_get(_params_handles.yaw_rate_ff_fw, &v);
	_params.rate_ff_fw(2) = v;
	// 90 ~ 0
	param_get(_params_handles.vt_tilt_1_mc, &v);
	_params.vt_tilt_1_mc = v;
	param_get(_params_handles.vt_tilt_1_fw, &v);
	_params.vt_tilt_1_fw = v;
	param_get(_params_handles.vt_tilt_2_mc, &v);
	_params.vt_tilt_2_mc = v;
	param_get(_params_handles.vt_tilt_2_fw, &v);
	_params.vt_tilt_2_fw = v;
	param_get(_params_handles.vt_tilt_3_mc, &v);
	_params.vt_tilt_3_mc = v;
	param_get(_params_handles.vt_tilt_3_fw, &v);
	_params.vt_tilt_3_fw = v;
	param_get(_params_handles.vt_tilt_4_mc, &v);
	_params.vt_tilt_4_mc = v;
	param_get(_params_handles.vt_tilt_4_fw, &v);
	_params.vt_tilt_4_fw = v;

	return OK;
}

// 大量消息的订阅↓↓↓↓↓

// 地面站参数的订阅，并存入变量
void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

// 飞行器状态
// 根据是不是在vtol飞行器下，改变角速度期望、多轴控制量的ID
void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

// 电机的限制
void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
	}
}

void
MulticopterAttitudeControl::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void
MulticopterAttitudeControl::v44_tilt_flag_poll()
{
	bool updated;
	orb_check(_v44_tilt_flag_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(v44_tilt_flag), _v44_tilt_flag_sub, &_v44_tilt_flag);
	}
}

// 大量信息的订阅↑↑↑↑↑

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	// 更新姿态期望
	// 这个期望是由速度与位置环根据控制模式得到的
	vehicle_attitude_setpoint_poll();
	_thrust_sp = _v_att_sp.thrust; //总的拉力期望值，在计算姿态期望时得到
	                               //！！！

	/* construct attitude setpoint rotation matrix */
	// 姿态期望点旋转矩阵
	math::Quaternion q_sp(_v_att_sp.q_d[0], _v_att_sp.q_d[1], _v_att_sp.q_d[2], _v_att_sp.q_d[3]);
	math::Matrix<3, 3> R_sp = q_sp.to_dcm();
	/* get current rotation matrix from control state quaternions */
	// 姿态旋转矩阵
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();

	/* all input data is ready, run controller itself */

	/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
	// 首先，考虑期望拉力和实际拉力（Z轴）的夹角，以及旋转轴
	math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
	math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

	/* axis and sin(angle) of desired rotation */
	math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z); //(R_z % R_sp_z)计算出旋转轴
	                                                       //R.transposed()旋转到机体轴系下

	/* calculate angle error */
	float e_R_z_sin = e_R.length(); //这是一个绝对值，会导致下面atan2f求得的角度总在0～180度之间
	float e_R_z_cos = R_z * R_sp_z;

	/* calculate weight for yaw control */
	float yaw_w = R_sp(2, 2) * R_sp(2, 2);

	/* calculate rotation matrix after roll/pitch only rotation */
	math::Matrix<3, 3> R_rp;

	if (e_R_z_sin > 0.0f) {
		/* get axis-angle representation */
		float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos); //夹角，对应于roll和pitch的偏差
		math::Vector<3> e_R_z_axis = e_R / e_R_z_sin; //旋转轴归一化

		e_R = e_R_z_axis * e_R_z_angle; //进一步生成各个方向的角度偏差，旋转矢量

		/* cross product matrix for e_R_axis */
		math::Matrix<3, 3> e_R_cp;
		e_R_cp.zero();
		e_R_cp(0, 1) = -e_R_z_axis(2);
		e_R_cp(0, 2) = e_R_z_axis(1);
		e_R_cp(1, 0) = e_R_z_axis(2);
		e_R_cp(1, 2) = -e_R_z_axis(0);
		e_R_cp(2, 0) = -e_R_z_axis(1);
		e_R_cp(2, 1) = e_R_z_axis(0);

		/* rotation matrix for roll/pitch only rotation */
		// 这个旋转矩阵认为roll和pitch已经旋转完成了
		// 通过旋转矢量计算完成roll和pitch旋转的旋转矩阵，再投影到惯性系下
		// see “Quaternion kinematics for the error-state KF”
		R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

	} else {
		/* zero roll/pitch rotation */
		R_rp = R;
	}

	/* R_rp and R_sp has the same Z axis, calculate yaw error */
	// 计算航向误差
	math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
	math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
	e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w; //(R_rp_x % R_sp_x)的方向与R_sp_z方向重合
	                                                                      //(R_rp_x % R_sp_x).length()总是正的，计算的航向误差应该在-180～180度之间

	// 拉力方向相差角度较大，大于90度
	if (e_R_z_cos < 0.0f) {
		/* for large thrust vector rotations use another rotation method:
		 * calculate angle and axis for R -> R_sp rotation directly */
		math::Quaternion q_error;
		q_error.from_dcm(R.transposed() * R_sp); //四元数的差
		math::Vector<3> e_R_d = q_error(0) >= 0.0f ? q_error.imag()  * 2.0f : -q_error.imag() * 2.0f; //虚部×2，准确来说应该是2asin(虚部)

		/* use fusion of Z axis based rotation and direct rotation */
		float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
		e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
	}

	/* calculate angular rates setpoint */
	// 计算角速度的期望值，比例控制
	// ！！！
	_rates_sp = _params.att_p.emult(e_R); //p、q、r的期望值

	/* limit rates */
	// 对角速度大小进行限制，3个通道
	for (int i = 0; i < 3; i++) {
		if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
		    !_v_control_mode.flag_control_manual_enabled) { //如果在速度环控制或者auto模式下
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.auto_rate_max(i), _params.auto_rate_max(i));
		} else { //如果是在手动控制下，采用不同门限限幅
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}
	}

	/* feed forward yaw setpoint rate */
	// 为了实现航向的准确跟踪，加前馈
	_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

	/* weather-vane mode, dampen yaw rate */
	// 如果抑制了航向的控制
	if ((_v_control_mode.flag_control_velocity_enabled || _v_control_mode.flag_control_auto_enabled) &&
	    _v_att_sp.disable_mc_yaw_control == true && !_v_control_mode.flag_control_manual_enabled) { //不是在手动控制下
		float wv_yaw_rate_max = _params.auto_rate_max(2) * _params.vtol_wv_yaw_rate_scale; //缩放比默认是0.15
		_rates_sp(2) = math::constrain(_rates_sp(2), -wv_yaw_rate_max, wv_yaw_rate_max); //重新设置门限
		// prevent integrator winding up in weathervane mode
		_rates_int(2) = 0.0f;
		_rates_int_fw(2) = 0.0f;
	}
}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	// 为了保证解锁/进入旋翼模式后，姿态角控制的积分器为0
	if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
		_rates_int_fw.zero();
	}

	/* current body angular rates */
	// 一个3维列向量
	math::Vector<3> rates;
	rates(0) = _ctrl_state.roll_rate;
	rates(1) = _ctrl_state.pitch_rate;
	rates(2) = _ctrl_state.yaw_rate;

	/* throttle pid attenuation factor */
	// 一般情况下拉力在0～1之间，tpa的值为0
	float tpa =  fmaxf(0.0f, 
	                   fminf(1.0f, 
					         1.0f - _params.tpa_slope * (fabsf(_v_rates_sp.thrust) - _params.tpa_breakpoint)));

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;

	// 列向量元素相乘
	// 滚转、俯仰、偏航的角速度控制
	// ！！！
	// 存放的是MC模式下的控制量
	_att_control = _params.rate_p.emult(rates_err * tpa) + _params.rate_d.emult(_rates_prev - rates) / dt + _rates_int +
		           _params.rate_ff.emult(_rates_sp);
	// 存放FW模式下的控制量
	_att_control_ts = _params.rate_p_fw.emult(rates_err * tpa) +
					  _params.rate_d_fw.emult(_rates_prev - rates) / dt +
					  _rates_int_fw + _params.rate_ff_fw.emult(_rates_sp);

	_rates_sp_prev = _rates_sp; //
	_rates_prev = rates; //用于下一控制循环

	// 旋翼倾转90度所对应的PWM的范围
	float MF_PWM_range = (fabsf(_params.vt_tilt_1_mc - _params.vt_tilt_1_fw) +
						  fabsf(_params.vt_tilt_2_mc - _params.vt_tilt_2_fw) +
						  fabsf(_params.vt_tilt_3_mc - _params.vt_tilt_3_fw) +
						  fabsf(_params.vt_tilt_4_mc - _params.vt_tilt_4_fw)) / 4.0f;
	// ！！！
	// 根据V44的控制，重新分配控制量
	// ！！！
	// PX4_INFO("MC control: %.5f, %.5f, %.5f, %.5f \n", (double)_thrust_sp, (double)_att_control(0),
														// (double)_att_control(1), (double)_att_control(2));
	// PX4_INFO("FW control: %.5f, %.5f, %.5f \n", (double)_att_control_ts(0),
												// (double)_att_control_ts(1), (double)_att_control_ts(2));
	float In = M_PI_2_F - _v44_tilt_flag.tilt_angle; //注意PWM增加的方向与旋翼倾转角度相反
	// MC滚转和FW航向耦合，左右旋翼差动
	float _att_control_0 = sinf(In) * _att_control(0) + cosf(In) * _att_control_ts(2);
	float _att_control_ts_0 = 0;
	// 俯仰
	float _att_control_1 = sinf(In) * _att_control(1);
	float _att_control_ts_1 = -cosf(In) * _att_control_ts(1);
	// 航向和滚转耦合，左右倾转轴差动
	float _att_control_2 = 0;
	float _att_control_ts_2 = -cosf(In) * _att_control_ts(0) + sinf(In) * _att_control(2);
	// 赋值
	_att_control(0) = _att_control_0;       //电机控制
	_att_control(1) = _att_control_1;
	_att_control(2) = _att_control_2;
	_att_control_ts(0) = _att_control_ts_0; //舵机控制
	_att_control_ts(1) = _att_control_ts_1;
	_att_control_ts(2) = _att_control_ts_2;
	// 4个倾转轴
	_ts_1234(0) = _v44_tilt_flag.tilt_angle / M_PI_2_F * (_params.vt_tilt_1_fw - _params.vt_tilt_1_mc) + _params.vt_tilt_1_mc;
	_ts_1234(1) = _v44_tilt_flag.tilt_angle / M_PI_2_F * (_params.vt_tilt_2_fw - _params.vt_tilt_2_mc) + _params.vt_tilt_2_mc;
	_ts_1234(2) = _v44_tilt_flag.tilt_angle / M_PI_2_F * (_params.vt_tilt_3_fw - _params.vt_tilt_3_mc) + _params.vt_tilt_3_mc;
	_ts_1234(3) = _v44_tilt_flag.tilt_angle / M_PI_2_F * (_params.vt_tilt_4_fw - _params.vt_tilt_4_mc) + _params.vt_tilt_4_mc;

	/* update integral only if not saturated on low limit and if motor commands are not saturated */
	// 如果拉力期望值大于最小起飞拉力，且电机没有饱和
	if (_thrust_sp > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit) {
		int i;
		i = 0; //对应于左右旋翼差动（MC滚转控制、FW航向控制）
		if (fabsf(_att_control(i)) < _thrust_sp){
			float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;
			float rate_i_fw = _rates_int_fw(i + 2) + _params.rate_i_fw(i + 2) * rates_err(i + 2) * dt;
			if (PX4_ISFINITE(rate_i) &&
				rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				_att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
				_rates_int(i) = rate_i;
			}
			if (PX4_ISFINITE(rate_i_fw) &&
				rate_i_fw > -RATES_I_LIMIT && rate_i_fw < RATES_I_LIMIT &&
				_att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
				_rates_int_fw(i + 2) = rate_i_fw;
			}
		}
		i = 1; //对应于俯仰控制（双权控制）
		if (fabsf(_att_control(i)) < _thrust_sp) {
			float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;
			if (PX4_ISFINITE(rate_i) &&
				rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				_att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
				_rates_int(i) = rate_i;
			}
		}
		float rate_i_fw = _rates_int_fw(i) + _params.rate_i_fw(i) * rates_err(i) * dt;
		if (PX4_ISFINITE(rate_i_fw) && 
			rate_i_fw > -RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			rate_i_fw < RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			_att_control_ts(i) > -RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			_att_control_ts(i) < RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range){
			_rates_int_fw(i) = rate_i_fw;
		}
		i = 2; //对应于左右倾转轴差动
		float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;
		rate_i_fw = _rates_int_fw(i - 2) + _params.rate_i_fw(i - 2) * rates_err(i - 2) * dt;
		if (PX4_ISFINITE(rate_i) && 
			rate_i > -RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			rate_i < RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			_att_control_ts(i) > -RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			_att_control_ts(i) < RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range){
			_rates_int(i) = rate_i;
		}
		if (PX4_ISFINITE(rate_i_fw) && 
			rate_i_fw > -RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			rate_i_fw < RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			_att_control_ts(i) > -RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range &&
			_att_control_ts(i) < RATES_I_LIMIT_TS * _v44_tilt_flag.max_tilt_angle / M_PI_2_F * MF_PWM_range){
			_rates_int_fw(i - 2) = rate_i_fw;
		}
		// 飞行速度较低时，固定翼控制器的积分分离
		if (_v44_tilt_flag.tilt_mode == MC_MODE ||
		    _v44_tilt_flag.tilt_mode == TRANSITION_FRONT_P1 ||
			_v44_tilt_flag.tilt_mode == TRANSITION_BACK_P2)
			_rates_int_fw.zero();
		// 飞行速度较快时，多旋翼控制器的积分分离
		if (_v44_tilt_flag.tilt_mode == FW_MODE ||
		    _v44_tilt_flag.tilt_mode == TRANSITION_FRONT_P2 ||
			_v44_tilt_flag.tilt_mode == TRANSITION_BACK_P1)
			_rates_int.zero();
	}
}

// 做一个中转
void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control::g_control->task_main();
}

//
// ！！！主要的任务执行函数
//
/*
内环控制——
	1. 判断是否在rattitude模式下，如果是，需要在杆量较大时禁能姿态控制
	2. 如果进行姿态控制，会生成角速度期望_rates_sp（见// ！！！），并由姿态期望获得总的拉力期望_thrust_sp
	   姿态期望由位置控制的线程发布
	   （立式起降飞机，需要有“最优恢复控制策略”）
	3. 如果不进行姿态控制，并且在手动控制下，那么就是要进行acro模式控制，那么需要将杆量直接对应于_rates_sp和_thrust_sp
	   如果不处于手动控制，那说明需要由别的线程发布期望值
	4. 进行角速度控制（见// ！！！），并将姿态控制量_att_control和拉力期望_thrust_sp发布给混控
*/
void
MulticopterAttitudeControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	_v44_tilt_flag_sub = orb_subscribe(ORB_ID(v44_tilt_flag));

	/* initialize parameters cache */ //初始化参数缓存
	parameters_update();

	/* wakeup source: vehicle attitude */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _ctrl_state_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		// 等待超过100ms使用continue是为了检测_task_should_exit
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("mc att ctrl: poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf); //开启循环计时器

		/* run controller on attitude changes */
		// 事件触发，如果句柄_ctrl_state_sub所示消息更新了
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			// 循环的时间间隔，以s为单位
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();
			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			// 更新控制所需要的全部状态
			/* copy attitude and control state topics */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();
			battery_status_poll();
			v44_tilt_flag_poll();

			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers */
			if (_v_control_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}

			//
			// 姿态控制
			//
			if (_v_control_mode.flag_control_attitude_enabled) {
				if (_ts_opt_recovery == nullptr) { //不会用到立式起降飞机的“最优恢复控制策略”
					// the  tailsitter recovery instance has not been created, thus, the vehicle
					// is not a tailsitter, do normal attitude control
					control_attitude(dt); //直接生成角速度期望_rates_sp，并获得拉力期望_thrust_sp
				} else { //使用到立式起降飞机的“最优恢复控制策略”
					vehicle_attitude_setpoint_poll();
					_thrust_sp = _v_att_sp.thrust;
					math::Quaternion q(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
					math::Quaternion q_sp(&_v_att_sp.q_d[0]);
					_ts_opt_recovery->setAttGains(_params.att_p, _params.yaw_ff);
					_ts_opt_recovery->calcOptimalRates(q, q_sp, _v_att_sp.yaw_sp_move_rate, _rates_sp);

					/* limit rates */
					for (int i = 0; i < 3; i++) {
						_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
					}
				}
				/* publish attitude rates setpoint */
				// 发布姿态角速度期望，当然前面也订阅的一个姿态角速度，这是由于不同模式的控制目的
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}
			} else { //如果不进行姿态控制，那么姿态角速度期望可能是由遥控器发出，或者某一个线程发布
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {//如果手动的话，说明在acro模式或者rattitude模式下
					/* manual rates control - ACRO mode */
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
								    _manual_control_sp.r).emult(_params.acro_rate_max); //相乘，杆量的值限制在了-1～1之间
					_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_MULTICOPTER);

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}
				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			// 
			// 姿态角速度控制
			//
			// 姿态控制是所有控制的最内环，不可能被禁能
			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				/* publish actuator controls 0 */
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.control[7] = _v_att_sp.landing_gear; //起落架
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _ctrl_state.timestamp;
				/* publish actuator controls 1 */
				_actuators1.control[0] = (PX4_ISFINITE(_att_control_ts(0))) ? _att_control_ts(0) : 0.0f;
				_actuators1.control[1] = (PX4_ISFINITE(_att_control_ts(1))) ? _att_control_ts(1) : 0.0f;
				_actuators1.control[2] = (PX4_ISFINITE(_att_control_ts(2))) ? _att_control_ts(2) : 0.0f;
				_actuators1.control[3] = 0;
				_actuators1.control[4] = (PX4_ISFINITE(_ts_1234(0))) ? _ts_1234(0) : 0.0f;
				_actuators1.control[5] = (PX4_ISFINITE(_ts_1234(1))) ? _ts_1234(1) : 0.0f;
				_actuators1.control[6] = (PX4_ISFINITE(_ts_1234(2))) ? _ts_1234(2) : 0.0f;
				_actuators1.control[7] = (PX4_ISFINITE(_ts_1234(3))) ? _ts_1234(3) : 0.0f;
				_actuators1.timestamp = hrt_absolute_time();
				_actuators1.timestamp_sample = _ctrl_state.timestamp;

				/* scale effort by battery status */
				// 根据设置，需要由电池状态缩放控制量
				if (_params.bat_scale_en && _battery_status.scale > 0.0f) {
					for (int i = 0; i < 4; i++) {
						_actuators.control[i] *= _battery_status.scale;
						_actuators1.control[i] *= _battery_status.scale;
					}
				}

				// 记录控制器状态
				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();
				// 记录v44控制状态
				_v44_control_status.tilt_mode = _v44_tilt_flag.tilt_mode;
				_v44_control_status.max_tilt_angle = _v44_tilt_flag.max_tilt_angle;
				_v44_control_status.tilt_angle = _v44_tilt_flag.tilt_angle;
				_v44_control_status.lon_velocity = _v44_tilt_flag.lon_velocity;
				_v44_control_status.thrust_sp = _thrust_sp;
				_v44_control_status.left_right_rotor = _att_control(0);
				_v44_control_status.forw_back_rotor = _att_control(1);
				_v44_control_status.servo1 = _ts_1234(0);
				_v44_control_status.servo2 = _ts_1234(1);
				_v44_control_status.servo3 = _ts_1234(2);
				_v44_control_status.servo4 = _ts_1234(3);
				_v44_control_status.left_right_servo = _att_control_ts(2);
				_v44_control_status.forw_back_servo = _att_control_ts(1);
				_v44_control_status.timestamp = hrt_absolute_time();

				PX4_INFO("V44 data: %.5f, %.5f, %.5f \n", (double)_v44_control_status.tilt_mode,
		                                                  (double)_actuators1.control[4], 
														  (double)_actuators1.control[2]);

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) { //发布控制量给混控！！！
						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);
					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}
				}
				if (_actuators_1_pub != nullptr) {
					orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators1);
				} else {
					_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators1);
				}

				/* publish controller status */
				// 发布控制器状态
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);
				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
				if (_v44_control_status_pub != nullptr) {
					orb_publish(ORB_ID(v44_control_status), _v44_control_status_pub, &_v44_control_status);
				} else {
					_v44_control_status_pub = orb_advertise(ORB_ID(v44_control_status), &_v44_control_status);
				}
			}
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
	return;
}

int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&MulticopterAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

// 
// 开启一个线程 or 关闭一个线程
// 
int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_att_control {start|stop|status}");
		return 1;
	}

	// 开启一个线程
	if (!strcmp(argv[1], "start")) {

		if (mc_att_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		// 赋值一个新的class，先运行构造函数
		mc_att_control::g_control = new MulticopterAttitudeControl;

		if (mc_att_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != mc_att_control::g_control->start()) { //运行start()函数
			delete mc_att_control::g_control;
			mc_att_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	// 关闭一个线程
	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		// 删除一个class对象，运行析构函数
		delete mc_att_control::g_control;
		mc_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
