/****************************************************************************
 *
 *   Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 *
 * Original publication for the desired attitude generation:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/manual_control_setpoint.h>
// #include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h> //根据是不是vtol，决定姿态角期望的发送位置
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h> //3个一组
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/v44_tilt_flag.h>

#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ONE_G	9.8066f

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControl();
	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	// 交叉范围线？？？
	bool	cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
					          const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);

private:
	bool	_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */ //mavlink日志发布，实现在地面站上的日志显示

	// 关于订阅
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */ //似乎没用到
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */ //似乎没用到

	// 关于发布
	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */
	orb_advert_t    _v44_tilt_flag_sp_pub;
	orb_id_t _attitude_setpoint_id; //姿态期望发布的目标ID

	// V44 倾转
	enum vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
		FW_MODE,			    /**< vtol is in fixed wing mode */
		TRANSITION_BACK 		/**< vtol is in back transition mode */
	};
	struct {
		vtol_mode flight_mode;			/**< vtol flight mode, defined by enum vtol_mode */
		hrt_abstime transition_start;	/**< absoulte time at which front transition started */
	} _vtol_schedule;

	// 结构体，要订阅或者发布的各种消息
	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */
	struct v44_tilt_flag_s _v44_tilt_flag_sp;

	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;

	// 微分
	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	// 地面站参数句柄
	struct {
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t alt_ctl_dz;
		param_t alt_ctl_dy;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_xy_dz;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t acc_hor_max;
		param_t alt_mode;
		param_t opt_recover;

	}		_params_handles;		/**< handles for interesting parameters */

	// 存储地面站参数
	struct {
		float thr_min;
		float thr_max;
		float thr_hover;
		float alt_ctl_dz;
		float alt_ctl_dy;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_xy_dz;
		float hold_max_xy;
		float hold_max_z;
		float acc_hor_max;
		float vel_max_up;
		float vel_max_down;
		uint32_t alt_mode;

		int opt_recover;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> vel_cruise;
		math::Vector<3> sp_offs_max;
	}		_params;

	// 地图上的参考位置
	struct map_projection_reference_s _ref_pos; //经纬度
	float _ref_alt; //高度
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _do_reset_alt_pos_flag;
	bool _mode_auto;
	// 下面的标志位主要在期望值计算时起到调整作用
	bool _pos_hold_engaged; //位置保持工作中，主要是手动模式下的标志位
	bool _alt_hold_engaged;
	bool _run_pos_control; //运行位置控制，主要是自动模式下的标志位
	bool _run_alt_control;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;	    /**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */

	math::Matrix<3, 3> _R;	  /**< rotation matrix from attitude quaternions */ //姿态四元数旋转矩阵
	float _yaw;				  /**< yaw angle (euler) */
	bool _in_landing;	      /**< the vehicle is in the landing descent */
	bool _lnd_reached_ground; /**< controller assumes the vehicle has reached the ground after landing */
	bool _takeoff_jumped;
	float _vel_z_lp; //lp，低通
	float _acc_z_lp;
	float _takeoff_thrust_sp;
	bool control_vel_enabled_prev;	/**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled) */

	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	// 位置、速度状态重置计数
	uint8_t _z_reset_counter;
	uint8_t _xy_reset_counter;
	uint8_t _vz_reset_counter;
	uint8_t _vxy_reset_counter;
	uint8_t _heading_reset_counter;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void	control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void	poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz, float dy); //缩放控制？？？
	static float    throttle_curve(float ctl, float ctr);

	/**
	 * Update reference for local position projection
	 */
	void	update_ref();

	/**
	 * Reset position setpoint to current position.
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 */
	 // 根据_reset_pos_sp值，重置位置期望
	 // 首先在飞行过程中激活这个标志位，然后切换到位置控制需要保持住最近的位置
	void	reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 */
	 // 根据_reset_alt_sp值，重置位置期望
	void	reset_alt_sp();

	/**
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 */
	 // 当位置期望距离太远，需要限制一下
	void	limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 */
	void	control_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void	control_offboard(float dt);

	/**
	 * Set position setpoint for AUTO
	 */
	void	control_auto(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	 // 气压计高度 or AMSL高度
	void	select_alt(bool global);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void		task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void	task_main();
};

// 命名空间，定义了一个class变量
namespace pos_control
{
	MulticopterPositionControl	*g_control;
}

// 构造函数
MulticopterPositionControl::MulticopterPositionControl() :
	SuperBlock(NULL, "MPC"), //保证class变量的定义
	_task_should_exit(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),

	/* subscriptions */
	// 关于订阅
	_ctrl_state_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),

	/* publications */
	// 关于发布
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_v44_tilt_flag_sp_pub(nullptr),
	_attitude_setpoint_id(0),

	// 存储所需的结构体
	_vehicle_status{},
	_vehicle_land_detected{},
	_ctrl_state{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_arming{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_global_vel_sp{},
	_v44_tilt_flag_sp{},

	// 都是Block相关的变量
	_manual_thr_min(this, "MANTHR_MIN"), //最小的手动控制油门
	_manual_thr_max(this, "MANTHR_MAX"), //最大的手动控制油门
	_vel_x_deriv(this, "VELD"), //为了计算速度微分
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),

	// 变量初始化
	_ref_alt(0.0f),
	_ref_timestamp(0),
	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_do_reset_alt_pos_flag(true),
	_mode_auto(false),
	_pos_hold_engaged(false),
	_alt_hold_engaged(false),
	_run_pos_control(true),
	_run_alt_control(true),
	_yaw(0.0f),
	_in_landing(false),
	_lnd_reached_ground(false),
	_takeoff_jumped(false),
	_vel_z_lp(0),
	_acc_z_lp(0),
	_takeoff_thrust_sp(0.0f),
	control_vel_enabled_prev(false),
	_z_reset_counter(0),
	_xy_reset_counter(0),
	_vz_reset_counter(0),
	_vxy_reset_counter(0),
	_heading_reset_counter(0)
{
	// V44倾转变量
	_vtol_schedule.flight_mode = MC_MODE; //默认是在多轴模式下
	_vtol_schedule.transition_start = 0;

	// Make the quaternion valid for control state
	_ctrl_state.q[0] = 1.0f;
	// 各个变量初始化
	memset(&_ref_pos, 0, sizeof(_ref_pos));
	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_cruise.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();
	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();
	_R.identity(); //先赋值为单位矩阵

	// 各个参数的句柄
	_params_handles.thr_min		= param_find("MPC_THR_MIN"); //auto控制下的最小拉力 0.12
	_params_handles.thr_max		= param_find("MPC_THR_MAX"); //auto控制下的最大拉力 0.9
	_params_handles.thr_hover	= param_find("MPC_THR_HOVER"); //旋停拉力 0.5
	_params_handles.alt_ctl_dz	= param_find("MPC_ALTCTL_DZ"); //定高拉力曲线断点 0.1，范围为center-dz to center+dz
	_params_handles.alt_ctl_dy	= param_find("MPC_ALTCTL_DY"); //上述范围对应的油门高度 0（代表一个死区）
	_params_handles.z_p			= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP"); //最大上升速度
	_params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX"); //最大下降速度

	// transitional support: Copy param values from max to down
	// param so that max param can be renamed in 1-2 releases
	// (currently at 1.3.0)
	// 过渡性支持，不要在意
	float p;
	param_get(param_find("MPC_Z_VEL_MAX"), &p);
	param_set(param_find("MPC_Z_VEL_MAX_DN"), &p);

	_params_handles.z_ff		= param_find("MPC_Z_FF"); //前馈
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX"); //最大的水平面速度 8
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE"); //auto模式下的一般巡航速度，也是进入位置辅助的速度 5
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR"); //auto和位置辅助下的飞机最大倾转角度 45
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tko_speed	= param_find("MPC_TKO_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND"); //降落过程中的最大倾转角度 15
	_params_handles.man_roll_max = param_find("MPC_MAN_R_MAX"); //手动飞时的最大姿态角
	_params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX"); //手动飞时的最大航向角速度 200
	_params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX"); //在mc_att_control_params.c
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P"); //在mc_att_control_params.c
	_params_handles.hold_xy_dz = param_find("MPC_HOLD_XY_DZ"); //位置保持时的X、Y杆死区
	_params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY"); //位置保持时的最大水平速度 0.8
	_params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z"); //位置保持时的最大垂向速度 0.6
	_params_handles.acc_hor_max = param_find("MPC_ACC_HOR_MAX"); //速度模式下的最大水平加速度
	_params_handles.alt_mode = param_find("MPC_ALT_MODE"); //高度保持 or 地形保持
	_params_handles.opt_recover = param_find("VT_OPT_RECOV_EN"); //定义在vtol_att_control_params.c

	/* fetch initial parameter values */
	parameters_update(true);
}

// 析构函数
MulticopterPositionControl::~MulticopterPositionControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true; //该赋值会导致跳出task_main()中的大循环

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
		} while (_control_task != -1); //跳出循环时，_control_task==-1
	}

	pos_control::g_control = nullptr;
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;
	orb_check(_params_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) { //地面站更新了控制参数，或者需要强制更新参数
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.thr_hover, &_params.thr_hover);
		param_get(_params_handles.alt_ctl_dz, &_params.alt_ctl_dz);
		param_get(_params_handles.alt_ctl_dy, &_params.alt_ctl_dy);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air); //转化为弧度制
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tko_speed, &_params.tko_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);

		float v;
		uint32_t v_i;
		param_get(_params_handles.xy_p, &v); //位置控制参数P
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_vel_p, &v); //速度控制参数P
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v); //I
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v); //D
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v); //速度限幅
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_max_up = v;
		_params.vel_max(2) = v;
		param_get(_params_handles.z_vel_max_down, &v);
		_params.vel_max_down = v;
		param_get(_params_handles.xy_vel_cruise, &v);
		_params.vel_cruise(0) = v;
		_params.vel_cruise(1) = v;
		/* using Z max up for now */
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_cruise(2) = v;
		param_get(_params_handles.xy_ff, &v); //位置控制前馈
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;
		param_get(_params_handles.hold_xy_dz, &v); //位置保持时的X、Y杆死区
		v = math::constrain(v, 0.0f, 1.0f);
		_params.hold_xy_dz = v;
		param_get(_params_handles.hold_max_xy, &v); //位置保持时的最大水平速度
		_params.hold_max_xy = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.hold_max_z, &v); //位置保持时的最大水平速度
		_params.hold_max_z = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.acc_hor_max, &v); //速度模式下的最大水平加速度
		_params.acc_hor_max = v;
		/*
		 * increase the maximum horizontal acceleration such that stopping
		 * within 1s from full speed is feasible
		 */
		_params.acc_hor_max = math::max(_params.vel_cruise(0), _params.acc_hor_max); //取最大值
		param_get(_params_handles.alt_mode, &v_i);
		_params.alt_mode = v_i;

		int i;
		param_get(_params_handles.opt_recover, &i);
		_params.opt_recover = i;

		_params.sp_offs_max = _params.vel_cruise.edivide(_params.pos_p) * 2.0f; //除以

		/* mc attitude control parameters*/
		/* manual control scale */
		param_get(_params_handles.man_roll_max, &_params.man_roll_max); //姿态（期望）限幅
		param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
		_params.man_roll_max = math::radians(_params.man_roll_max); //转化为弧度
		_params.man_pitch_max = math::radians(_params.man_pitch_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		_params.global_yaw_max = math::radians(_params.global_yaw_max);

		param_get(_params_handles.mc_att_yaw_p, &v); //航向角控制的P
		_params.mc_att_yaw_p = v;

		/* takeoff and land velocities should not exceed maximum */
		_params.tko_speed = fminf(_params.tko_speed, _params.vel_max_up);
		_params.land_speed = fminf(_params.land_speed, _params.vel_max_down);
	}

	return OK;
}

// 获得订阅，更新变量
void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	// 更新飞行器状态
	orb_check(_vehicle_status_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_attitude_setpoint_id) { //只有初始时，根据是否在vtol确定姿态期望的发布方向
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);
			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}

	// 落地检测
	orb_check(_vehicle_land_detected_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	// 控制状态
	orb_check(_ctrl_state_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
		/* get current rotation matrix and euler angles from control state quaternions */
		math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		_yaw = euler_angles(2); //航向角
		if (_control_mode.flag_control_manual_enabled) { //手动控制
			if (_heading_reset_counter != _ctrl_state.quat_reset_counter) { //机头方向重置
				_heading_reset_counter = _ctrl_state.quat_reset_counter;
				math::Quaternion delta_q(_ctrl_state.delta_q_reset[0], _ctrl_state.delta_q_reset[1],
				                         _ctrl_state.delta_q_reset[2], _ctrl_state.delta_q_reset[3]);
				// we only extract the heading change from the delta quaternion
				math::Vector<3> delta_euler = delta_q.to_euler();
				_att_sp.yaw_body += delta_euler(2);
			}
		}

	}

	// 姿态期望，可能直接由某些线程发布过来，可能性较小
	orb_check(_att_sp_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	// 控制模式
	orb_check(_control_mode_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	// 手动，控制器输入
	orb_check(_manual_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	// 飞行器armed状态
	orb_check(_arming_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	// 本地位置
	orb_check(_local_pos_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
		// check if a reset event has happened
		// if the vehicle is in manual mode we will shift the setpoints of the
		// states which were reset. In auto mode we do not shift the setpoints
		// since we want the vehicle to track the original state.
		if (_control_mode.flag_control_manual_enabled) {
			if (_z_reset_counter != _local_pos.z_reset_counter) { //位置重置
				_pos_sp(2) += _local_pos.delta_z;
			}
			if (_xy_reset_counter != _local_pos.xy_reset_counter) {
				_pos_sp(0) += _local_pos.delta_xy[0];
				_pos_sp(1) += _local_pos.delta_xy[1];
			}
			if (_vz_reset_counter != _local_pos.vz_reset_counter) {
				_vel_sp(2) += _local_pos.delta_vz;
				_vel_sp_prev(2) +=  _local_pos.delta_vz;
			}
			if (_vxy_reset_counter != _local_pos.vxy_reset_counter) {
				_vel_sp(0) += _local_pos.delta_vxy[0];
				_vel_sp(1) += _local_pos.delta_vxy[1];
				_vel_sp_prev(0) += _local_pos.delta_vxy[0];
				_vel_sp_prev(1) += _local_pos.delta_vxy[1];
			}
		}
		// update the reset counters in any case
		_z_reset_counter = _local_pos.z_reset_counter;
		_xy_reset_counter = _local_pos.xy_reset_counter;
		_vz_reset_counter = _local_pos.vz_reset_counter;
		_vxy_reset_counter = _local_pos.vxy_reset_counter;
	}
}

float
MulticopterPositionControl::scale_control(float ctl, float end, float dz, float dy)
// ctl：油门杆量-0.5  保证油门杆量在-0.5～0.5之间，中间杆量对应就是0
// end：0.5
// dz：油门死区范围
// dy：死区范围的油门高度
// 相当于将油门杆量映射到一个正负范围内，并且该范围内有一个死区
// 主要用于垂向速度控制
{
	if (ctl > dz) {
		return dy + (ctl - dz) * (1.0f - dy) / (end - dz);
	} else if (ctl < -dz) {
		return -dy + (ctl + dz) * (1.0f - dy) / (end - dz);
	} else {
		return ctl * (dy / dz);
	}
}

float
MulticopterPositionControl::throttle_curve(float ctl, float ctr)
{
	/* piecewise linear mapping: 0:ctr -> 0:0.5
	 * and ctr:1 -> 0.5:1 */
	// 一般ctl表示油门杆量，ctr表示旋停油门额定值
	// 下面的操作，保证了油门杆在中间时，刚好达到旋停油门值
	if (ctl < 0.5f) {
		return 2 * ctl * ctr;
	} else {
		return ctr + 2 * (ctl - 0.5f) * (1.0f - ctr);
	}
}

// 起到中转作用
void
MulticopterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
}

// 根据预留时间戳，更新位置期望值
// 更新_pos_sp
void
MulticopterPositionControl::update_ref()
{
	// 更新了新的飞行器位置信息
	if (_local_pos.ref_timestamp != _ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			// 位置期望转化到经纬高坐标系下
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
			// 获得lat_sp、lon_sp、alt_sp
		}

		/* update local projection reference */
		// 获得飞行原点所在经纬度_ref_pos、海拔_ref_alt，作为参考信号
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			// 获得新坐标系下的位置期望
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
			// 获得列向量_pos_sp
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

// 重置位置期望、高度期望
// 直接给期望赋值为当前位置
void
MulticopterPositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;
		// we have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
	}
}
void
MulticopterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;
		// we have logic in the main function which choosed the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// altitude in a special way
		_pos_sp(2) = _pos(2);
	}
}

// 限制位置期望补偿
// 当位置期望与实际位置相差很大时，对位置期望值做个缩放
// 防止出现对于较大位置期望的跟踪
void
MulticopterPositionControl::limit_pos_sp_offset()
{
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0); //(_pos_sp(0) - _pos(0))*_params.pos_p/(_params.vel_cruise*2.0f)
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}
	if (_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();
	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}

//
// 手动控制
//
/*
	1. 由auto进入手动控制，做好位置、高度复位的准备
	2. 根据杆量生成速度期望，垂向速度期望有死区（通过scale_control(...)函数实现）
	3. 将上诉速度期望进行归一化，并乘以额定巡航速度，转换到NED坐标系下
	4. 如果进行“位置”控制，首先根据前面的设置进行单次的位置期望复位
			如果杆量足够小，当位置控制没有被占用，当水平面速度足够小时——占用位置控制，设置当前位置为位置期望
											当水平速度较大时——不占用位置控制，飞机会慢慢减速直到进入位置控制
			如果杆量太大，不占用位置控制
	   当位置控制不占用时，_pos_sp(0~1)设置为当前位置（没有位置环），杆量直接对应于速度期望_vel_sp(0~1)
	   _run_pos_control表示是否进行着位置控制
	5. 如果进行“高度”控制，首先根据前面的设置进行单次的高度期望复位
			如果油门杆在死区内，当高度控制没有被占用，当垂向速度足够小时——占用高度控制，设置当前高度为高度期望
												当垂向速度较大时——不占用高度控制，飞机会减速进入高度控制
			如果杆量不在死区内，不占用高度控制
	   当高度控制不占用时，_pos_sp(2)设置为当前高度（没有高度控制），杆量直接对应于速度期望_vel_sp(2)
	   _run_alt_control表示是否进行着高度控制
	6. 此函数，根据控制模式，填入了位置或速度期望_pos_sp、_vel_sp，根据条件，关闭了控制标志位_run_pos_control、_run_alt_control
*/
void
MulticopterPositionControl::control_manual(float dt)
{
	/* Entering manual control from non-manual control mode, reset alt/pos setpoints */
	// 从auto进入手动，需要为重置位置期望点做好准备
	// 因为手动下有高度和位置辅助
	if (_mode_auto) {
		_mode_auto = false;
		/* Reset alt pos flags if resetting is enabled */
		if (_do_reset_alt_pos_flag) {
			_reset_pos_sp = true;
			_reset_alt_sp = true;
		}
	}

	// 速度期望值 + 位置期望复位
	math::Vector<3> req_vel_sp; // X,Y in local frame and Z in global (D), in [-1,1] normalized range
	req_vel_sp.zero();
	if (_control_mode.flag_control_altitude_enabled) {
		/* set vertical velocity setpoint with throttle stick */
		req_vel_sp(2) = -scale_control(_manual.z - 0.5f, 0.5f, _params.alt_ctl_dz, _params.alt_ctl_dy); // D
	}
	if (_control_mode.flag_control_position_enabled) {
		/* set horizontal velocity setpoint with roll/pitch stick */
		req_vel_sp(0) = _manual.x;
		req_vel_sp(1) = _manual.y;
	}
	if (_control_mode.flag_control_altitude_enabled) {
		/* reset alt setpoint to current altitude if needed */
		reset_alt_sp();
	}
	if (_control_mode.flag_control_position_enabled) {
		/* reset position setpoint to current position if needed */
		reset_pos_sp();
	}
	/* limit velocity setpoint */
	// 限制速度期望值
	float req_vel_sp_norm = req_vel_sp.length();
	if (req_vel_sp_norm > 1.0f) {
		req_vel_sp /= req_vel_sp_norm;
	}

	/* _req_vel_sp scaled to 0～1, scale it to max speed and rotate around yaw */
	// 由于受到航向角的影响，需要把机体轴系下的速度期望旋转到END坐标系下
	math::Matrix<3, 3> R_yaw_sp;
	R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
	math::Vector<3> req_vel_sp_scaled = R_yaw_sp * req_vel_sp.emult(_params.vel_cruise); // in NED and scaled to actual velocity

	/*
	 * assisted velocity mode: user controls velocity, but if	velocity is small enough, position
	 * hold is activated for the corresponding axis
	 */
	/* horizontal axes */
	if (_control_mode.flag_control_position_enabled) { //位置辅助
		/* check for pos. hold */
		// 杆量给的速度期望是不是足够小
		if (fabsf(req_vel_sp(0)) < _params.hold_xy_dz && fabsf(req_vel_sp(1)) < _params.hold_xy_dz) {
			// 如果位置控制没有占用
			if (!_pos_hold_engaged) {
				float vel_xy_mag = sqrtf(_vel(0) * _vel(0) + _vel(1) * _vel(1));
				// 如果实际水平面速度小于一设定值
				// 就把当前的位置作为位置期望，并且只发布一次
				if (_params.hold_max_xy < FLT_EPSILON || vel_xy_mag < _params.hold_max_xy) {
					/* reset position setpoint to have smooth transition from velocity control to position control */
					_pos_hold_engaged = true;
					_pos_sp(0) = _pos(0);
					_pos_sp(1) = _pos(1);
				} else {
					_pos_hold_engaged = false;
				}
			}
		} else {
			_pos_hold_engaged = false;
		}
		/* set requested velocity setpoint */
		// 不然，把当前位置作为位置期望，相当于没有位置控制环
		// 速度期望由杆量给出
		if (!_pos_hold_engaged) {
			_pos_sp(0) = _pos(0);
			_pos_sp(1) = _pos(1);
			_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
			_vel_sp(0) = req_vel_sp_scaled(0);
			_vel_sp(1) = req_vel_sp_scaled(1);
		}
	}
	/* vertical axis */
	// 位置辅助下，一定有高度辅助
	// 根据高度速度杆量，生成高度期望（V44待修改）
	if (_control_mode.flag_control_altitude_enabled) { //高度辅助
		/* check for pos. hold */
		// 如果油门杆回中，在死区范围内
		if (fabsf(req_vel_sp(2)) < FLT_EPSILON) {
			if (!_alt_hold_engaged) {
				// 如果当前的速度足够小
				// 把当前的高度作为高度期望，并且只发布一次
				if (_params.hold_max_z < FLT_EPSILON || fabsf(_vel(2)) < _params.hold_max_z) {
					/* reset position setpoint to have smooth transition from velocity control to position control */
					_alt_hold_engaged = true;
					_pos_sp(2) = _pos(2);
				} else {
					_alt_hold_engaged = false;
				}
			}
		} else {
			_alt_hold_engaged = false;
			_pos_sp(2) = _pos(2);
		}
		/* set requested velocity setpoint */
		if (!_alt_hold_engaged) {
			_run_alt_control = false; /* request velocity setpoint to be used, instead of altitude setpoint */
			_vel_sp(2) = req_vel_sp_scaled(2);
		}
	}
}

//
// offboard模式
//
/*
	1. 订阅一个特殊的topic position_setpoint_triplet，获得位置期望
	2. 如果当前位置期望可用，
			（1）如果进行位置控制，获得水平位置期望
			    如果不进行位置控制，进行速度控制，根据需要复位位置期望，获得速度期望，关闭标志位_run_pos_control
			（2）一定存在航向控制，或是直接获取航向期望，或是跟自己航向期望角速度计算，直接赋值_att_sp.yaw_body
			（3）如果进行高度控制，获得高度期望
			    如果不进行高度控制，进行climb_rate控制，根据需要复位高度期望，获得速度期望，关闭标志位_run_alt_control
	   如果当前位置期望不可用，
	   		根据设定，复位位置、高度期望
	3. 此函数，根据控制模式，填入了位置或速度期望_pos_sp、_vel_sp，根据条件，关闭了控制标志位_run_pos_control、_run_alt_control
	          填入航向期望值_att_sp.yaw_body
*/
void
MulticopterPositionControl::control_offboard(float dt)
{
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
	// 订阅一个特殊的位置期望点topic

	if (_pos_sp_triplet.current.valid) {
		// 位置 & 速度控制，NED坐标系下的期望值
		if (_control_mode.flag_control_position_enabled && _pos_sp_triplet.current.position_valid) {
			/* control position */
			_pos_sp(0) = _pos_sp_triplet.current.x;
			_pos_sp(1) = _pos_sp_triplet.current.y;
		} else if (_control_mode.flag_control_velocity_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* control velocity */
			/* reset position setpoint to current position if needed */
			// 这里之所以说如果需要，是因为并没有对_reset_pos_sp进行赋值
			reset_pos_sp();
			/* set position setpoint move rate */
			_vel_sp(0) = _pos_sp_triplet.current.vx;
			_vel_sp(1) = _pos_sp_triplet.current.vy;
			_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
		}

		// 一定存在的航向角度控制
		if (_pos_sp_triplet.current.yaw_valid) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		} else if (_pos_sp_triplet.current.yawspeed_valid) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;
		}

		// 保持固定的高度
		if (_control_mode.flag_control_altitude_enabled && _pos_sp_triplet.current.alt_valid) {
			/* control altitude as it is enabled */
			_pos_sp(2) = _pos_sp_triplet.current.z;
			_run_alt_control = true;
		} else if (_control_mode.flag_control_altitude_enabled && _pos_sp_triplet.current.position_valid) {
			/* control altitude because full position control is enabled */
			_pos_sp(2) = _pos_sp_triplet.current.z;
			_run_alt_control = true;
		} else if (_control_mode.flag_control_climb_rate_enabled && _pos_sp_triplet.current.velocity_valid) {
			// 起飞时，直接给的是速度期望
			/* reset alt setpoint to current altitude if needed */
			reset_alt_sp();
			/* set altitude setpoint move rate */
			_vel_sp(2) = _pos_sp_triplet.current.vz;
			_run_alt_control = false; /* request velocity setpoint to be used, instead of position setpoint */
		}
	} else {
		reset_pos_sp();
		reset_alt_sp();
	}
}

// 当前位置、参数、前参考点、当前参考点 -> 动态参考点
// ？？？
bool
MulticopterPositionControl::cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
		                                      const math::Vector<3> line_a, const math::Vector<3> line_b,
											        math::Vector<3> &res)
{
	/* project center of sphere on line */
	/* normalized AB */
	math::Vector<3> ab_norm = line_b - line_a;
	ab_norm.normalize();
	math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		/* we have triangle CDX with known CD and CX = R, find DX */
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			/* target waypoint is already behind us */
			res = line_b;
		} else {
			/* target is in front of us */
			res = d + ab_norm * dx_len; // vector A->B on line
		}
		return true;
	} else {
		/* have no roots, return D */
		res = d; /* go directly to line */
		/* previous waypoint is still in front of us */
		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}
		/* target waypoint is already behind us */
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}
		return false;
	}
}

//
// auto模式
//
/*
	1. 如果首次进入auto模式，进行单次位置、高度期望复位
	   如果在非旋翼模式下，要持续进行复位
	2. 更新topic position_setpoint_triplet，并判断其是否可用
	3. 如果当前期望位置可用，转换到NED坐标系下curr_sp，且current_setpoint_valid赋值true，表示当前位置期望可用
	   如果上一期望位置可用，转换到NED坐标系下prev_sp，且previous_setpoint_valid赋值true，表示上一位置期望可用
	4. 通过地面站参数或者之前订阅的topic更新巡航速度，并定义缩放比例scale对curr_sp进行缩放
	5. 如果在航迹点飞行或目标跟踪下前一个位置期望点可用，且当前与前一个位置期望点之间的距离较大时，
			如果当前位置缩放与当前期望缩放相距较小，生成一个动态的缩放期望点pos_sp_s
			如果当前位置缩放与当前期望缩放相距较大，采取另一种方式生成pos_sp_s（通过cross_sphere_line(...)函数实现）
	6. 基于上一动态的缩放期望点，生成动态位置期望_pos_sp
	7. 利用航向角速度期望或者航向期望值，更新航向期望_att_sp.yaw_body
	8. 在某些情况下将_do_reset_alt_pos_flag赋值为false，以实现切换至手动后不发生震荡
	9. 起飞、降落时，保证起落架放下
	10. 此函数，填入了位置期望_pos_sp和航向期望值_att_sp.yaw_body
	    根据条件，关闭了标志位_do_reset_alt_pos_flag
*/
void MulticopterPositionControl::control_auto(float dt)
{
	/* reset position setpoint on AUTO mode activation or if we are not in MC mode */
	if (!_mode_auto || !_vehicle_status.is_rotary_wing) {
		if (!_mode_auto) {
			_mode_auto = true;
		}
		_reset_pos_sp = true;
		_reset_alt_sp = true;
	}
	// Always check reset state of altitude and position control flags in auto
	reset_pos_sp();
	reset_alt_sp();

	// Poll position setpoint
	// 更新位置期望，与offboard模式不同，这里获得的是经纬高值
	// 因为auto模式是由地面站给发送位置期望的
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
		//Make sure that the position setpoint is valid
		if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.lon) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}
	}

	bool current_setpoint_valid = false;
	bool previous_setpoint_valid = false;
	math::Vector<3> prev_sp;
	math::Vector<3> curr_sp; //当前位置期望

	if (_pos_sp_triplet.current.valid) {
		/* project setpoint to local frame */
		// 位置期望映射到控制坐标系中
		map_projection_project(&_ref_pos, _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
		                       &curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);
		if (PX4_ISFINITE(curr_sp(0)) &&
		    PX4_ISFINITE(curr_sp(1)) &&
		    PX4_ISFINITE(curr_sp(2))) {
			current_setpoint_valid = true;
		}
	}
	// 上一时刻的位置期望
	if (_pos_sp_triplet.previous.valid) {
		map_projection_project(&_ref_pos, _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon,
				               &prev_sp.data[0], &prev_sp.data[1]);
		prev_sp(2) = -(_pos_sp_triplet.previous.alt - _ref_alt);
		if (PX4_ISFINITE(prev_sp(0)) &&
		    PX4_ISFINITE(prev_sp(1)) &&
		    PX4_ISFINITE(prev_sp(2))) {
			previous_setpoint_valid = true;
		}
	}

	if (current_setpoint_valid && //如果期望点可用
	    (_pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_IDLE)) {
		/* scaled space: 1 == position error resulting max allowed speed */

		math::Vector<3> cruising_speed = _params.vel_cruise;
		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
		    _pos_sp_triplet.current.cruising_speed > 0.1f) { //期望点可以发送巡航速度信息
			cruising_speed(0) = _pos_sp_triplet.current.cruising_speed;
			cruising_speed(1) = _pos_sp_triplet.current.cruising_speed;
		}

		math::Vector<3> scale = _params.pos_p.edivide(cruising_speed);
		/* convert current setpoint to scaled space */
		math::Vector<3> curr_sp_s = curr_sp.emult(scale); //当前位置期望值×P/设定巡航速度
		/* by default use current setpoint as is */
		math::Vector<3> pos_sp_s = curr_sp_s; //动态期望点缩放

		// 如果是航迹点飞行或者跟踪目标
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION  ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) &&
		    previous_setpoint_valid) {
			/* follow "previous - current" line */
			// 跟随期望点之间的线
			if ((curr_sp - prev_sp).length() > MIN_DIST) {
				/* find X - cross point of unit sphere and trajectory */
				math::Vector<3> pos_s = _pos.emult(scale); //当前位置的缩放
				math::Vector<3> prev_sp_s = prev_sp.emult(scale); //前一个期望点的缩放
				math::Vector<3> prev_curr_s = curr_sp_s - prev_sp_s; //动态期望点缩放-前一个期望点的缩放
				math::Vector<3> curr_pos_s = pos_s - curr_sp_s; //当前位置的缩放-动态期望点缩放
				float curr_pos_s_len = curr_pos_s.length();
				if (curr_pos_s_len < 1.0f) {
					/* copter is closer to waypoint than unit radius */
					/* check next waypoint and use it to avoid slowing down when passing via waypoint */
					if (_pos_sp_triplet.next.valid) { //如果下一个位置期望点可用
						math::Vector<3> next_sp;
						map_projection_project(&_ref_pos, _pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon,
								               &next_sp.data[0], &next_sp.data[1]);
						next_sp(2) = -(_pos_sp_triplet.next.alt - _ref_alt);
						if ((next_sp - curr_sp).length() > MIN_DIST) {
							math::Vector<3> next_sp_s = next_sp.emult(scale); //获得下一个位置期望点，并进行缩放
							/* calculate angle prev - curr - next */
							math::Vector<3> curr_next_s = next_sp_s - curr_sp_s; //下一个期望点的缩放-动态期望点缩放
							math::Vector<3> prev_curr_s_norm = prev_curr_s.normalized();
							// ？？？
							/* cos(a) * curr_next, a = angle between current and next trajectory segments */
							float cos_a_curr_next = prev_curr_s_norm * curr_next_s;
							/* cos(b), b = angle pos - curr_sp - prev_sp */
							float cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;
							if (cos_a_curr_next > 0.0f && cos_b > 0.0f) {
								float curr_next_s_len = curr_next_s.length();
								/* if curr - next distance is larger than unit radius, limit it */
								if (curr_next_s_len > 1.0f) {
									cos_a_curr_next /= curr_next_s_len;
								}
								/* feed forward position setpoint offset */
								math::Vector<3> pos_ff = prev_curr_s_norm *
											 cos_a_curr_next * cos_b * cos_b * (1.0f - curr_pos_s_len) *
											 (1.0f - expf(-curr_pos_s_len * curr_pos_s_len * 20.0f));
								pos_sp_s += pos_ff; //获得动态期望点缩放
							}
						}
					}
				} else {
					bool near = cross_sphere_line(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);
					if (!near) {
						/* we're far away from trajectory, pos_sp_s is set to the nearest point on the trajectory */
						pos_sp_s = pos_s + (pos_sp_s - pos_s).normalized();
					}
				}
			}
		}
		// 上面的内容，是根据航迹点位置（前一个、当前、下一个）计算出动态期望点的缩放值
		/* move setpoint not faster than max allowed speed */
		math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale); 
		/* difference between current and desired position setpoints, 1 = max speed */
		math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
		float d_pos_m_len = d_pos_m.length();
		if (d_pos_m_len > dt) {
			pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
		}
		/* scale result back to normal space */
		_pos_sp = pos_sp_s.edivide(scale); //实际的新的参考点的值

		/* update yaw setpoint if needed */
		// 更新航向角期望值
		if (_pos_sp_triplet.current.yawspeed_valid
		    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;
		} else if (PX4_ISFINITE(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		}

		/*
		 * if we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		                                                            不能对高度或者位置控制进行reset
		 * this makes the takeoff finish smoothly.
		 */
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
		     || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)
		    && _pos_sp_triplet.current.acceptance_radius > 0.0f
		    /* need to detect we're close a bit before the navigator switches from takeoff to next waypoint */
		    && (_pos - _pos_sp).length() < _pos_sp_triplet.current.acceptance_radius * 1.2f) {
			_do_reset_alt_pos_flag = false;
			/* otherwise: in case of interrupted mission don't go to waypoint but stay at current position */
		} else {
			_do_reset_alt_pos_flag = true;
		}

		// During a mission or in loiter it's safe to retract the landing gear
		// 收放起落架
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
		    !_vehicle_land_detected.landed) {
			_att_sp.landing_gear = 1.0f;
		// During takeoff and landing, we better put it down again.
		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			       _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			_att_sp.landing_gear = -1.0f;
		} else {
			// For the rest of the setpoint types, just leave it as is.
		}
	} else {
		/* no waypoint, do nothing, setpoint was already reset */
	}
}

// 上面的函数control_manual(float dt)、control_offboard(float dt)、control_auto(float dt)
// 根据不同的控制模式生成了“NED坐标系”下的位置期望_pos_sp、速度期望_vel_sp、_att_sp中的航向角期望
//
// ！！！主任务执行程序
//
/*
外环控制——
	1. 主要更新topic vehicle_local_position 获得飞机但前的位置，并更新其他的topic和控制参数
	2. 如果是首次armed，进行多个复位的允许，包括位置期望、高度期望、手动控制下的复位操作、xyz3个方向的拉力积分、航向期望
	                  上一速度期望设置为0
	   如果在vtol的固定翼模式下，允许高度期望、航向期望复位
	3. 利用update_ref()更新飞行原点的参考位置经纬高_ref_pos、_ref_alt
	   将_pos_sp中的位置期望转化到新的坐标系下，经纬高->NED（似乎并没有什么用，后面会更新位置期望）
	4. 更新飞机的NED位置和速度信息_pos、_vel，并计算速度微分
	5. 对手动模式下是否占用位置、高度控制进行设置，实时检测是否退出占用
	6. 如果进行外环（位置+速度）控制，
		（1）默认_run_pos_control、_run_alt_control都是true的，之后根据控制模式调用control_manual(float dt)、
		    control_offboard(float dt)、control_auto(float dt)函数，产生期望值，并对上面2个标志位更新
		（2）判断是否禁能航向控制_att_sp.disable_mc_yaw_control，会作用于内环控制中
		（3）如果没有在手动控制且正在怠速等待，需要设定恒定的姿态、拉力期望
		（4）如果在手动控制且在地面上，需要设定恒定的姿态、拉力期望，
		                          进行多个复位的允许，包括位置期望、高度期望、手动控制下的复位操作、xyz3个方向的拉力积分，
	                              auto模式标志置为false
	7. 如果进行正常的飞行控制，
		位置环生成速度期望（见// ！！！）
			（1）如果_run_pos_control被置位，要生成水平速度期望_vel_sp(0~1)
			    如果计算的速度期望和给定的速度期望都可用，且处于目标跟踪时，要保证速度期望不小于设定期望的一个缩放值
				如果当前位置不可用，需要直接赋值一个速度期望
			（2）如果_run_alt_control被置位，需要生成垂向速度期望_vel_sp(2)，并对其进行限幅
			（3）如果一些控制模式被禁能，要做好使能准备，允许一些复位操作并初始化参数
			（4）如果是自主降落，采用恒定的速度
				如果是自主起飞，首先需要飞机“跳”起来，直接给_takeoff_thrust_sp赋一个不断上升的值，速度期望设置为0，直到
				上升速度大于恒定爬升速度的一半，认为“跳”起来了，并为z方向的拉力积分器赋值
				之后，采用恒定的爬升速度
			（5）利用加速度进一步限制速度期望，并将速度期望发布
		如果进行速度控制、加速度控制、climb_rate控制，要由速度环生成姿态期望（见// ！！！）
			（1）先对xyz3个方向的拉力积分器复位，z方向的复位还要判断reset_int_z_manual，以保证手动到定高的稳定
			（2）如果上一时刻的速度期望位置，也就是上一时刻没有进入速度环控制，要对当前的速度期望进行调整
			（3）计算加速度期望作为拉力期望_thrust_sp，并根据飞行模式进行调整
			    例如，起飞时，要将上面列向量置0,并使_thrust_sp(2)=-takeoff_thrust_sp
			（4）对3个方向的拉力期望进行限幅，并计算总的拉力期望thrust_abs，如果拉力期望没有饱和，会更新拉力积分器
			（5）如果进行了速度或加速度控制，要由上述拉力期望计算姿态期望，赋值给_att_sp.q_d，赋值_att_sp.thrust=thrust_abs用于内环控制
	8. 如果不进行外环（位置+速度）控制，进行多个复位的允许，包括位置期望、高度期望、手动控制下的复位操作、xyz3个方向的拉力积分，
	                               auto模式标志置为false，上一速度期望设置为当前速度，
								   利用上一速度期望进行控制的标志位设置为false，作为是否首次进入速度控制的标志
	9. 在上面复位允许的基础上，如果是手动控制下的姿态控制，那么需要由杆量生成姿态期望
			（1）armed后，要进行单次的航向复位
				如果在天上，或进行了高度控制，或有油门杆量，航向期望_att_sp.yaw_sp根据杆量对应的航向角速度设定
			（2）如果不进行climb_rate控制，直接把油门杆量给_att_sp.thrust
			（3）如果不进行速度控制，_att_sp.roll_body、_att_sp.pitch_body直接对应杆量，并转换为_att_sp.q_d用于内环控制
			（4）收放起落架
	10. 如果在offboard模式下，且位置控制或速度控制或加速度控制没有被使能，就不会发布姿态期望，否则发布_att_sp。
*/
void
MulticopterPositionControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status)); //无人机状态，是不是处于vtol或是处于旋翼等
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state)); //用于控制的系统状态
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint)); //姿态期望的订阅基本用不到
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position)); //本地位置，经纬高、DED等
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet)); //offboeard、auto模式下的位置期望
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

	parameters_update(true); //true代表强制更新来自地面站的参数
	/* initialize values of critical structs until first regular update */
	_arming.armed = false;
	/* get an initial update for all sensor and status data */
	poll_subscriptions(); //更新各个topic
	/* We really need to know from the beginning if we're landed or in-air. */
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool reset_yaw_sp = true;
	bool was_armed = false;

	hrt_abstime t_prev = 0; //用于计算控制周期

	math::Vector<3> thrust_int; //拉力积分值
	thrust_int.zero();

	// Let's be safe and have the landing gear down by default
	_att_sp.landing_gear = -1.0f;

	matrix::Dcmf R;
	R.identity();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) { //进入用于位置控制的大循环
		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);
		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
		}
		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions(); //更新状态
		parameters_update(false); //更新参数

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f; //外环控制周期
		t_prev = t;
		// set dt for control blocks
		setDt(dt);

		// 一些reset标志的初始化
		// 位置reset实际上就是把当前位置赋值给期望值
		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_do_reset_alt_pos_flag = true;
			_vel_sp_prev.zero();
			reset_int_z = true;
			reset_int_xy = true;
			reset_yaw_sp = true;
		}
		/* reset yaw and altitude setpoint for VTOL which are in fw mode */
		if (_vehicle_status.is_vtol) {
			if (!_vehicle_status.is_rotary_wing) {
				reset_yaw_sp = true;
				_reset_alt_sp = true;
			}
		}
		// Update previous arming state
		was_armed = _control_mode.flag_armed;

		// 更新_pos_sp，实际上是经纬高到NED的转化
		update_ref();

		/* Update velocity derivative,
		 * independent of the current flight mode
		 */
		if (_local_pos.timestamp > 0) {
			if (PX4_ISFINITE(_local_pos.x) &&
			    PX4_ISFINITE(_local_pos.y) &&
			    PX4_ISFINITE(_local_pos.z)) {
				_pos(0) = _local_pos.x;
				_pos(1) = _local_pos.y;
				if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
					_pos(2) = -_local_pos.dist_bottom; //保持与地形的相对高对
				} else {
					_pos(2) = _local_pos.z; //保持绝对高度
				}
			}
			if (PX4_ISFINITE(_local_pos.vx) &&
			    PX4_ISFINITE(_local_pos.vy) &&
			    PX4_ISFINITE(_local_pos.vz)) {
				_vel(0) = _local_pos.vx;
				_vel(1) = _local_pos.vy;
				if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
					_vel(2) = -_local_pos.dist_bottom_rate;
				} else {
					_vel(2) = _local_pos.vz;
				}
			}
			_vel_err_d(0) = _vel_x_deriv.update(-_vel(0)); //速度微分
			_vel_err_d(1) = _vel_y_deriv.update(-_vel(1));
			_vel_err_d(2) = _vel_z_deriv.update(-_vel(2));
		}

		// reset the horizontal and vertical position hold flags for non-manual modes
		// or if position / altitude is not controlled
		// 如果赋值为false，说明相应的期望值就等于实际值
		if (!_control_mode.flag_control_position_enabled || !_control_mode.flag_control_manual_enabled) {
			_pos_hold_engaged = false;
		}
		if (!_control_mode.flag_control_altitude_enabled || !_control_mode.flag_control_manual_enabled) {
			_alt_hold_engaged = false;
		}

		// 要求进行外环的控制 ↓
		if (_control_mode.flag_control_altitude_enabled || //需要高度控制（V44待修改）
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_acceleration_enabled) {
			_vel_ff.zero();

			/* by default, run position/altitude controller. the control_* functions
			 * can disable this and run velocity controllers directly in this cycle */
			// 每次控制循环里都是默认把位置控制和高度控制打开的
			// 如果有需要，会在函数control_manual(float dt)、control_offboard(float dt)、control_auto(float dt)中关掉
			_run_pos_control = true;
			_run_alt_control = true;

			/* select control source */
			if (_control_mode.flag_control_manual_enabled) {
				/* manual control */
				control_manual(dt);
			} else if (_control_mode.flag_control_offboard_enabled) {
				/* offboard control */
				control_offboard(dt);
				_mode_auto = false;
			} else {
				/* AUTO */
				control_auto(dt);
			}

			/* weather-vane mode for vtol: disable yaw control */
			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.disable_mc_yaw_control == true) {
				_att_sp.disable_mc_yaw_control = true;
			} else {
				/* reset in case of setpoint updates */
				_att_sp.disable_mc_yaw_control = false;
			}

			// 没有进入手动控制，且处于怠速等待中，姿态 + 拉力期望
			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
			    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
				/* idle state, don't run controller and set zero thrust */
				R.identity();
				matrix::Quatf qd = R;
				memcpy(&_att_sp.q_d[0], qd.data(), sizeof(_att_sp.q_d));
				_att_sp.q_d_valid = true;
				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _yaw;
				_att_sp.thrust = 0.0f;
				_att_sp.timestamp = hrt_absolute_time();
				/* publish attitude setpoint */
				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}
			// 手动模式，并且在地上，姿态 + 拉力期望
			} else if (_control_mode.flag_control_manual_enabled
				       && _vehicle_land_detected.landed) {
				/* don't run controller when landed */
				_reset_pos_sp = true;
				_reset_alt_sp = true;
				_do_reset_alt_pos_flag = true;
				_mode_auto = false;
				reset_int_z = true;
				reset_int_xy = true;
				R.identity();
				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _yaw;
				_att_sp.thrust = 0.0f;
				_att_sp.timestamp = hrt_absolute_time();
				/* publish attitude setpoint */
				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}
			} else {
				// 正常飞行时生成速度期望 ↓
				// ！！！
				/* run position & altitude controllers, if enabled (otherwise use already computed velocity setpoints) */
				
				// 如果要位置控制的话，那么虽然上面可能生成了速度期望值，但是需要由位置环生成速度期望
				if (_run_pos_control) {
					_vel_sp(0) = (_pos_sp(0) - _pos(0)) * _params.pos_p(0);
					_vel_sp(1) = (_pos_sp(1) - _pos(1)) * _params.pos_p(1);
				}
				// guard against any bad velocity values
				bool velocity_valid = PX4_ISFINITE(_pos_sp_triplet.current.vx) &&
						              PX4_ISFINITE(_pos_sp_triplet.current.vy) &&
						                           _pos_sp_triplet.current.velocity_valid;
				// do not go slower than the follow target velocity when position tracking is active (set to valid)
				// 位置跟踪时，不能比下面的目标速度慢
				if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET && //跟踪航迹点
				    velocity_valid && _pos_sp_triplet.current.position_valid) { //速度可用 且 位置可用
					math::Vector<3> ft_vel(_pos_sp_triplet.current.vx, _pos_sp_triplet.current.vy, 0);
					float cos_ratio = (ft_vel * _vel_sp) / (ft_vel.length() * _vel_sp.length());
					// only override velocity set points when uav is traveling in same direction as target and vector component
					// is greater than calculated position set point velocity component
					if (cos_ratio > 0) {
						ft_vel *= (cos_ratio);
						// min speed a little faster than target vel
						ft_vel += ft_vel.normalized() * 1.5f;
					} else {
						ft_vel.zero();
					}
					_vel_sp(0) = fabs(ft_vel(0)) > fabs(_vel_sp(0)) ? ft_vel(0) : _vel_sp(0);
					_vel_sp(1) = fabs(ft_vel(1)) > fabs(_vel_sp(1)) ? ft_vel(1) : _vel_sp(1);
					// track target using velocity only
				} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
					       velocity_valid) { //位置期望不可用，直接跟随速度期望
					_vel_sp(0) = _pos_sp_triplet.current.vx;
					_vel_sp(1) = _pos_sp_triplet.current.vy;
				}

				// 单独的高度的P控制（V44待修改）
				if (_run_alt_control) {
					_vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);
				}
				// 确定期望速度不超过设定的额定值
				/* make sure velocity setpoint is saturated in xy*/
				float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) + _vel_sp(1) *   _vel_sp(1));
				if (vel_norm_xy > _params.vel_max(0)) {
					/* note assumes vel_max(0) == vel_max(1) */
					_vel_sp(0) = _vel_sp(0) * _params.vel_max(0) / vel_norm_xy;
					_vel_sp(1) = _vel_sp(1) * _params.vel_max(1) / vel_norm_xy;
				}
				/* make sure velocity setpoint is saturated in z*/
				// （V44待修改）
				if (_vel_sp(2) < -1.0f * _params.vel_max_up) {
					_vel_sp(2) = -1.0f * _params.vel_max_up;
				}
				if (_vel_sp(2) >  _params.vel_max_down) {
					_vel_sp(2) = _params.vel_max_down;
				}

				// 如果一些控制被禁能，要做好其被使能的准备
				if (!_control_mode.flag_control_position_enabled) {
					_reset_pos_sp = true;
				}
				if (!_control_mode.flag_control_altitude_enabled) {
					_reset_alt_sp = true;
				}
				if (!_control_mode.flag_control_velocity_enabled) {
					_vel_sp_prev(0) = _vel(0);
					_vel_sp_prev(1) = _vel(1);
					_vel_sp(0) = 0.0f;
					_vel_sp(1) = 0.0f;
					control_vel_enabled_prev = false;
				}
				if (!_control_mode.flag_control_climb_rate_enabled) {
					_vel_sp(2) = 0.0f;
				}

				/* use constant descend rate when landing, ignore altitude setpoint */
				// 降落的时候使用固定速度
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
				    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
					_vel_sp(2) = _params.land_speed;
				}
				/* special thrust setpoint generation for takeoff from ground */
				// 起飞任务
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
				    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
				    && _control_mode.flag_armed) {
					// check if we are not already in air.
					// if yes then we don't need a jumped takeoff anymore
					// 判断飞机是不是离开地面
					if (!_takeoff_jumped && !_vehicle_land_detected.landed && fabsf(_takeoff_thrust_sp) < FLT_EPSILON) {
						_takeoff_jumped = true;
					}
					// 首先需要飞机“跳”起来
					if (!_takeoff_jumped) {
						// ramp thrust setpoint up
						if (_vel(2) > -(_params.tko_speed / 2.0f)) {
							_takeoff_thrust_sp += 0.5f * dt; //起飞用的拉力是不断增大的，直到达到设定速度
							_vel_sp.zero();
							_vel_prev.zero();
						} else {
							// copter has reached our takeoff speed. split the thrust setpoint up
							// into an integral part and into a P part
							// 将拉力期望点分为积分部分和比例部分
							thrust_int(2) = _takeoff_thrust_sp - _params.vel_p(2) * fabsf(_vel(2));
							thrust_int(2) = -math::constrain(thrust_int(2), _params.thr_min, _params.thr_max);
							_vel_sp_prev(2) = -_params.tko_speed;
							_takeoff_jumped = true;
							reset_int_z = false;
						}
					}
					// “起跳”后，可以设置恒定的爬升速度
					if (_takeoff_jumped) {
						_vel_sp(2) = -_params.tko_speed;
					}
				} else { //如果不是起飞任务
					_takeoff_jumped = false;
					_takeoff_thrust_sp = 0.0f;
				}

				// 限制加速度，生成新的速度期望
				// limit total horizontal acceleration
				math::Vector<2> acc_hor;
				acc_hor(0) = (_vel_sp(0) - _vel_sp_prev(0)) / dt;
				acc_hor(1) = (_vel_sp(1) - _vel_sp_prev(1)) / dt;
				if ((acc_hor.length() > _params.acc_hor_max) & !_reset_pos_sp) {
					acc_hor.normalize();
					acc_hor *= _params.acc_hor_max;
					math::Vector<2> vel_sp_hor_prev(_vel_sp_prev(0), _vel_sp_prev(1));
					math::Vector<2> vel_sp_hor = acc_hor * dt + vel_sp_hor_prev;
					_vel_sp(0) = vel_sp_hor(0);
					_vel_sp(1) = vel_sp_hor(1);
				}
				// limit vertical acceleration
				// 限制垂向加速度（V44待修改）
				float acc_v = (_vel_sp(2) - _vel_sp_prev(2)) / dt;
				if ((fabsf(acc_v) > 2 * _params.acc_hor_max) & !_reset_alt_sp) {
					acc_v /= fabsf(acc_v);
					_vel_sp(2) = acc_v * 2 * _params.acc_hor_max * dt + _vel_sp_prev(2);
				}
				_vel_sp_prev = _vel_sp;
				_global_vel_sp.vx = _vel_sp(0);
				_global_vel_sp.vy = _vel_sp(1);
				_global_vel_sp.vz = _vel_sp(2);

				/* publish velocity setpoint */
				//  发布速度期望
				if (_global_vel_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);
				} else {
					_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
				}
				// 正常飞行时生成速度期望 ↑

				// ！！！
				if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_acceleration_enabled) {
					// 计算NED三轴方向的所需拉力值 ↓
					/* reset integrals if needed */
					if (_control_mode.flag_control_climb_rate_enabled) {
						if (reset_int_z) {
							reset_int_z = false;
							float i = _params.thr_min;
							if (reset_int_z_manual) {
								i = _params.thr_hover;
								if (i < _params.thr_min) {
									i = _params.thr_min;
								} else if (i > _params.thr_max) {
									i = _params.thr_max;
								}
							}
							thrust_int(2) = -i;
						}
					} else {
						reset_int_z = true;
					}
					if (_control_mode.flag_control_velocity_enabled) {
						if (reset_int_xy) {
							reset_int_xy = false;
							thrust_int(0) = 0.0f;
							thrust_int(1) = 0.0f;
						}
					} else {
						reset_int_xy = true;
					}

					/* velocity error */
					math::Vector<3> vel_err = _vel_sp - _vel;
					// check if we have switched from a non-velocity controlled mode into a velocity controlled mode
					// if yes, then correct xy velocity setpoint such that the attitude setpoint is continuous
					// 如果是直接切入速度控制，需要调整xy的速度期望值
					if (!control_vel_enabled_prev && _control_mode.flag_control_velocity_enabled) {
						matrix::Dcmf Rb = matrix::Quatf(_att_sp.q_d[0], _att_sp.q_d[1], _att_sp.q_d[2], _att_sp.q_d[3]);
						// choose velocity xyz setpoint such that the resulting thrust setpoint has the direction
						// given by the last attitude setpoint
						_vel_sp(0) = _vel(0) +
						             (-Rb(0,2) * _att_sp.thrust - thrust_int(0) - _vel_err_d(0) * _params.vel_d(0)) / _params.vel_p(0);
						_vel_sp(1) = _vel(1) + 
						             (-Rb(1,2) * _att_sp.thrust - thrust_int(1) - _vel_err_d(1) * _params.vel_d(1)) / _params.vel_p(1);
						_vel_sp(2) = _vel(2) + 
						             (-Rb(2,2) * _att_sp.thrust - thrust_int(2) - _vel_err_d(2) * _params.vel_d(2)) / _params.vel_p(2);
						_vel_sp_prev(0) = _vel_sp(0);
						_vel_sp_prev(1) = _vel_sp(1);
						_vel_sp_prev(2) = _vel_sp(2);
						control_vel_enabled_prev = true;
						// compute updated velocity error
						vel_err = _vel_sp - _vel;
					}

					/* thrust vector in NED frame */
					math::Vector<3> thrust_sp;
					if (_control_mode.flag_control_acceleration_enabled && _pos_sp_triplet.current.acceleration_valid) {
						thrust_sp = math::Vector<3>(_pos_sp_triplet.current.a_x, _pos_sp_triplet.current.a_y, _pos_sp_triplet.current.a_z);
					} else {
						thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d) + thrust_int; //PID控制器生成期望拉力值
					}
					// 起飞的时候，由速度环提供一个恒定的拉力，其余轴的拉力为0
					if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
					    && !_takeoff_jumped && !_control_mode.flag_control_manual_enabled) {
						// for jumped takeoffs use special thrust setpoint calculated above
						thrust_sp.zero();
						thrust_sp(2) = -_takeoff_thrust_sp;
					}
					// 没有速度控制时，不能产生拉力期望
					if (!_control_mode.flag_control_velocity_enabled && !_control_mode.flag_control_acceleration_enabled) {
						thrust_sp(0) = 0.0f;
						thrust_sp(1) = 0.0f;
					}
					// 不起飞时，也没有拉力期望
					if (!_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_acceleration_enabled) {
						thrust_sp(2) = 0.0f;
					}

					/* limit thrust vector and check for saturation */
					bool saturation_xy = false;
					bool saturation_z = false;
					/* limit min lift */
					// 获得拉力最小值
					float thr_min = _params.thr_min;
					if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) {
						/* don't allow downside thrust direction in manual attitude mode */
						thr_min = 0.0f;
					}
					float thrust_abs = thrust_sp.length();
					float tilt_max = _params.tilt_max_air;
					float thr_max = _params.thr_max;
					/* filter vel_z over 1/8sec */ //速度低通滤波器
					_vel_z_lp = _vel_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * _vel(2);
					/* filter vel_z change over 1/8sec */ //加速度低通滤波器
					float vel_z_change = (_vel(2) - _vel_prev(2)) / dt;
					_acc_z_lp = _acc_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * vel_z_change;
					/* adjust limits for landing mode */
					// 为降落模式获得拉力值最大值
					if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
					    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
						/* limit max tilt and min lift when landing */
						tilt_max = _params.tilt_max_land;
						if (thr_min < 0.0f) {
							thr_min = 0.0f;
						}
						/* descend stabilized, we're landing */
						// 判断有没有必要降落
						if (!_in_landing && !_lnd_reached_ground  && (float)fabs(_acc_z_lp) < 0.1f
						    && _vel_z_lp > 0.5f * _params.land_speed) {
							_in_landing = true;
						}
						/* assume ground, cut thrust */
						// 判断是不是落在地面上了
						if (_in_landing && _vel_z_lp < 0.1f) {
							thr_max = 0.0f;
							_in_landing = false;
							_lnd_reached_ground = true;
						}
						/* once we assumed to have reached the ground always cut the thrust.
							Only free fall detection below can revoke this
						*/
						// 判断是不是已经落在地面上了
						if (!_in_landing && _lnd_reached_ground) {
							thr_max = 0.0f;
						}
						/* if we suddenly fall, reset landing logic and remove thrust limit */
						if (_lnd_reached_ground
						    /* XXX: magic value, assuming free fall above 4m/s2 acceleration */
						    && (_acc_z_lp > 4.0f
							|| _vel_z_lp > 2.0f * _params.land_speed)) {
							thr_max = _params.thr_max;
							_in_landing = false;
							_lnd_reached_ground = false;
						}
					} else {
						_in_landing = false;
						_lnd_reached_ground = false;
					}

					// 对拉力进行限幅
					/* limit min lift */ //最小拉力限幅
					if (-thrust_sp(2) < thr_min) {
						thrust_sp(2) = -thr_min;
						saturation_z = true;
					}
					if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {
						if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
							/* absolute horizontal thrust */
							float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
							if (thrust_sp_xy_len > 0.01f) {
								/* max horizontal thrust for given vertical thrust*/
								float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

								if (thrust_sp_xy_len > thrust_xy_max) {
									float k = thrust_xy_max / thrust_sp_xy_len;
									thrust_sp(0) *= k;
									thrust_sp(1) *= k;
									saturation_xy = true;
								}
							}
						}
					}
					// 垂向速度控制时的拉力补偿
					if (_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_velocity_enabled) {
						/* thrust compensation when vertical velocity but not horizontal velocity is controlled */
						float att_comp;
						if (_R(2, 2) > TILT_COS_MAX) {
							att_comp = 1.0f / _R(2, 2);
						} else if (_R(2, 2) > 0.0f) {
							att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _R(2, 2) + 1.0f;
							saturation_z = true;
						} else {
							att_comp = 1.0f;
							saturation_z = true;
						}
						thrust_sp(2) *= att_comp;
					}
					/* limit max thrust */ //最大拉力限幅
					thrust_abs = thrust_sp.length(); /* recalculate because it might have changed */ //旋翼所惨生的总的拉力
					if (thrust_abs > thr_max) {
						if (thrust_sp(2) < 0.0f) {
							if (-thrust_sp(2) > thr_max) { //z方向拉力饱和
								/* thrust Z component is too large, limit it */
								thrust_sp(0) = 0.0f;
								thrust_sp(1) = 0.0f;
								thrust_sp(2) = -thr_max;
								saturation_xy = true;
								saturation_z = true;
							} else { //z方向拉力没有饱和，是xy方向饱和
								/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
								float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp(2) * thrust_sp(2)); //拉力余量
								float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
								float k = thrust_xy_max / thrust_xy_abs;
								thrust_sp(0) *= k;
								thrust_sp(1) *= k;
								saturation_xy = true;
							}
						} else {
							/* Z component is positive, going down, simply limit thrust vector */
							float k = thr_max / thrust_abs;
							thrust_sp *= k;
							saturation_xy = true;
							saturation_z = true;
						}
						thrust_abs = thr_max;
					}

					/* update integrals */ //更新拉力的积分器
					if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
						thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
					}
					if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;
						/* protection against flipping on ground when landing */
						if (thrust_int(2) > 0.0f) {
							thrust_int(2) = 0.0f;
						}
					}
					// 计算NED三轴方向的所需拉力值 ↑

					/* calculate attitude setpoint from thrust vector */
					// 根据拉力期望计算所需要的姿态角度期望 ！！！
					if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {
						// 速度控制 or 加速度哦控制，需要外环生成姿态期望
						
						/* desired body_z axis = -normalize(thrust_vector) */
						math::Vector<3> body_x;
						math::Vector<3> body_y;
						math::Vector<3> body_z;
						
						if (thrust_abs > SIGMA) { //总的旋翼拉力
							body_z = -thrust_sp / thrust_abs; //---
						} else {
							/* no thrust, set Z axis to safe value */
							body_z.zero();
							body_z(2) = 1.0f;
						}
						/* vector of desired yaw direction in XY plane, rotated by PI/2 */
						math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);
						if (fabsf(body_z(2)) > SIGMA) {
							/* desired body_x axis, orthogonal to body_z */
							body_x = y_C % body_z; //---
							/* keep nose to front while inverted upside down */
							if (body_z(2) < 0.0f) {
								body_x = -body_x;
							}
							body_x.normalize();
						} else {
							/* desired thrust is in XY plane, set X downside to construct correct matrix,
							 * but yaw component will not be used actually */
							body_x.zero();
							body_x(2) = 1.0f;
						}
						/* desired body_y axis */
						body_y = body_z % body_x; //---

						/* fill rotation matrix */
						for (int i = 0; i < 3; i++) {
							R(i, 0) = body_x(i);
							R(i, 1) = body_y(i);
							R(i, 2) = body_z(i);
						}
						/* copy quaternion setpoint to attitude setpoint topic */
						matrix::Quatf q_sp = R;
						memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
						_att_sp.q_d_valid = true;
						/* calculate euler angles, for logging only, must not be used for control */
						// 不要用于控制？？？
						matrix::Eulerf euler = R;
						_att_sp.roll_body = euler(0);
						_att_sp.pitch_body = euler(1);
						/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */
					} else if (!_control_mode.flag_control_manual_enabled) {
						/* autonomous altitude control without position control (failsafe landing),
						 * force level attitude, don't change yaw */
						R = matrix::Eulerf(0.0f, 0.0f, _att_sp.yaw_body);
						/* copy quaternion setpoint to attitude setpoint topic */
						matrix::Quatf q_sp = R;
						memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
						_att_sp.q_d_valid = true;
						_att_sp.roll_body = 0.0f;
						_att_sp.pitch_body = 0.0f;
					}
					_att_sp.thrust = thrust_abs; //外环计算的总拉力期望（V44待修改）

					/* save thrust setpoint for logging */
					_local_pos_sp.acc_x = thrust_sp(0) * ONE_G;
					_local_pos_sp.acc_y = thrust_sp(1) * ONE_G;
					_local_pos_sp.acc_z = thrust_sp(2) * ONE_G;
					_att_sp.timestamp = hrt_absolute_time();
				} else {
					reset_int_z = true;
				}
			}

			/* fill local position, velocity and thrust setpoint */
			_local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;
			_local_pos_sp.vx = _vel_sp(0);
			_local_pos_sp.vy = _vel_sp(1);
			_local_pos_sp.vz = _vel_sp(2);
			/* publish local position setpoint */
			if (_local_pos_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}
		// 要求进行外环的控制 ↑
		} else {
			/* position controller disabled, reset setpoints */
			_reset_alt_sp = true;
			_reset_pos_sp = true;
			_do_reset_alt_pos_flag = true;
			_mode_auto = false;
			reset_int_z = true;
			reset_int_xy = true;
			control_vel_enabled_prev = false;
			/* store last velocity in case a mode switch to position control occurs */
			_vel_sp_prev = _vel;
		}

		/* generate attitude setpoint from manual controls */
		if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {
			// 手动且姿态控制，直接遥控器生成姿态期望（V44待修改）

			/* reset yaw setpoint to current position if needed */
			if (reset_yaw_sp) {
				reset_yaw_sp = false;
				_att_sp.yaw_body = _yaw;
			}
			/* do not move yaw while sitting on the ground */
			else if (!_vehicle_land_detected.landed && //表示上天了
				     !(!_control_mode.flag_control_altitude_enabled && _manual.z < 0.1f)) {
				/* we want to know the real constraint, and global overrides manual */
				const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? _params.man_yaw_max : //取较小值
							                                                                _params.global_yaw_max;
				const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;
				_att_sp.yaw_sp_move_rate = _manual.r * yaw_rate_max; //因为杆量是0～1之间
				float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
				float yaw_offs = _wrap_pi(yaw_target - _yaw);
				// If the yaw offset became too big for the system to track stop
				// shifting it, only allow if it would make the offset smaller again.
				if (fabsf(yaw_offs) < yaw_offset_max ||
				    (_att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) ||
				    (_att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0)) {
					_att_sp.yaw_body = yaw_target; //杆量生成的航向期望（V44待修改）
				}
			}

			/* control throttle directly if no climb rate controller is active */
			if (!_control_mode.flag_control_climb_rate_enabled) {
				float thr_val = throttle_curve(_manual.z, _params.thr_hover); //杆量给到油门
				_att_sp.thrust = math::min(thr_val, _manual_thr_max.get());
				/* enforce minimum throttle if not landed */
				if (!_vehicle_land_detected.landed) {
					_att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get()); //杆量生成的油门（V44待修改）
				}
			}
			/* control roll and pitch directly if no aiding velocity controller is active */
			if (!_control_mode.flag_control_velocity_enabled) {
				/***********************************************************************************/

				/* V44倾转控制状态机 */
				bool fw_is_request;
				if(_manual.transition_switch == manual_control_setpoint_s::SWITCH_POS_ON)
					fw_is_request = true;
				else
					fw_is_request = false;
				// 计算飞机的纵向速度
				float longitudinalV = 0.0f;
				float vxvy_yaw = cosf(_yaw) * _local_pos.vx +sinf(_yaw) * _local_pos.vy;
				longitudinalV = sqrtf(vxvy_yaw * vxvy_yaw + _local_pos.vz * _local_pos.vz);
				if(fw_is_request)
				{
					_v44_tilt_flag_sp.tilt_angle = 0;
				}
				else if(!fw_is_request)
				{
					_v44_tilt_flag_sp.tilt_angle = 0;
				}

				// 根据处于的状态，决定杆量的生成量
				// 记录倾转的状态
				_att_sp.roll_body = _manual.y * _params.man_roll_max; //杆量给滚转、俯仰角度（V44待修改）
				_att_sp.pitch_body = -_manual.x * _params.man_pitch_max;
				if(_vtol_schedule.flight_mode == MC_MODE)
				{
					_v44_tilt_flag_sp.tilt_mode = MC_MODE;
					if(_manual.x >= 0)
					{
						_v44_tilt_flag_sp.tilt_angle = _manual.x * _params.man_pitch_max; //以弧度为单位
						_att_sp.pitch_body = 0;
					}
				}
				else
				{
					if(_vtol_schedule.flight_mode == TRANSITION_FRONT_P1)
						_v44_tilt_flag_sp.tilt_mode = TRANSITION_FRONT_P1;
					if(_vtol_schedule.flight_mode == TRANSITION_FRONT_P2)
						_v44_tilt_flag_sp.tilt_mode = TRANSITION_FRONT_P1;
					if(_vtol_schedule.flight_mode == FW_MODE)
						_v44_tilt_flag_sp.tilt_mode = TRANSITION_FRONT_P1;
					if(_vtol_schedule.flight_mode == TRANSITION_BACK)
						_v44_tilt_flag_sp.tilt_mode = TRANSITION_BACK;
				}
				_v44_tilt_flag_sp.max_tilt_angle = _params.man_pitch_max; //该变量和积分器的限幅相关
				_v44_tilt_flag_sp.lon_velocity = longitudinalV;
				_v44_tilt_flag_sp.pitch_sp = _att_sp.pitch_body;
				_v44_tilt_flag_sp.timestamp = hrt_absolute_time();

				/***********************************************************************************/
				/* only if optimal recovery is not used, modify roll/pitch */
				if (_params.opt_recover <= 0) {//避免航向误差的影响
					// construct attitude setpoint rotation matrix. modify the setpoints for roll
					// and pitch such that they reflect the user's intention even if a yaw error
					// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
					// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
					// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
					// heading of the vehicle.

					// calculate our current yaw error
					float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);
					// compute the vector obtained by rotating a z unit vector by the rotation
					// given by the roll and pitch commands of the user
					math::Vector<3> zB = {0, 0, 1};
					math::Matrix<3, 3> R_sp_roll_pitch;
					R_sp_roll_pitch.from_euler(_att_sp.roll_body, _att_sp.pitch_body, 0);
					math::Vector<3> z_roll_pitch_sp = R_sp_roll_pitch * zB;
					// transform the vector into a new frame which is rotated around the z axis
					// by the current yaw error. this vector defines the desired tilt when we look
					// into the direction of the desired heading
					math::Matrix<3, 3> R_yaw_correction;
					R_yaw_correction.from_euler(0.0f, 0.0f, -yaw_error);
					z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;
					// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
					// to calculate the new desired roll and pitch angles
					// R_tilt can be written as a function of the new desired roll and pitch
					// angles. we get three equations and have to solve for 2 unknowns
					_att_sp.pitch_body = asinf(z_roll_pitch_sp(0));
					_att_sp.roll_body = -atan2f(z_roll_pitch_sp(1), z_roll_pitch_sp(2));
				}
				/* copy quaternion setpoint to attitude setpoint topic */
				matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
				memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
				_att_sp.q_d_valid = true;
			}

			if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON &&
			    !_vehicle_land_detected.landed) {
				_att_sp.landing_gear = 1.0f; //收起起落架
			} else if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
				_att_sp.landing_gear = -1.0f; //放下起落架
			}
			_att_sp.timestamp = hrt_absolute_time();
		} else {
			reset_yaw_sp = true;
			_att_sp.yaw_sp_move_rate = 0.0f;
		}
		/* update previous velocity for velocity controller D part */
		_vel_prev = _vel;
		/* publish attitude setpoint
		 * Do not publish if offboard is enabled but position/velocity/accel control is disabled,
		 * in this case the attitude setpoint is published by the mavlink app.
		 * Also do not publish if the vehicle is a VTOL and it's just doing a transition
		 * (the VTOL attitude control module will generate attitude setpoints for the transition).
		 */
		if (!(_control_mode.flag_control_offboard_enabled &&
		      !(_control_mode.flag_control_position_enabled ||
			_control_mode.flag_control_velocity_enabled ||
			_control_mode.flag_control_acceleration_enabled))) {
			if (_att_sp_pub != nullptr) {
				orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);
			} else if (_attitude_setpoint_id) {
				_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
			}
			if (_v44_tilt_flag_sp_pub != nullptr){
				orb_publish(ORB_ID(v44_tilt_flag), _v44_tilt_flag_sp_pub, &_v44_tilt_flag_sp);
			} else {
				_v44_tilt_flag_sp_pub = orb_advertise(ORB_ID(v44_tilt_flag), &_v44_tilt_flag_sp);
			}
		}
		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled
				                                      && !_control_mode.flag_control_climb_rate_enabled;
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");
	_control_task = -1;
}

int
MulticopterPositionControl::start()
{
	ASSERT(_control_task == -1); //声称变量值

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1900,
					   (px4_main_t)&MulticopterPositionControl::task_main_trampoline,
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
int mc_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (pos_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}
		// 给变量赋值，新的指针地址
		// 先运行构造函数
		pos_control::g_control = new MulticopterPositionControl;
		if (pos_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}
		if (OK != pos_control::g_control->start()) { //类里的start()函数
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	// 关闭线程，删除变量
	// 运行析构函数
	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}
		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
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
