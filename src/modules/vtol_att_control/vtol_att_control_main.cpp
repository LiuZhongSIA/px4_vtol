/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file VTOL_att_control_main.cpp
 * Implementation of an attitude controller for VTOL airframes. This module receives data
 * from both the fixed wing- and the multicopter attitude controllers and processes it.
 * It computes the correct actuator controls depending on which mode the vehicle is in (hover,forward-
 * flight or transition). It also publishes the resulting controls on the actuator controls topics.
 *
 * @author Roman Bapst 		<bapstr@ethz.ch>
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler	<thomasgubler@gmail.com>
 * @author David Vorsin		<davidvorsin@gmail.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
 */
// Read 6

#include "vtol_att_control_main.h"
#include <systemlib/mavlink_log.h>

namespace VTOL_att_control
{
	VtolAttitudeControl *g_control;
}

/**
* Constructor
* 构造函数，类中元素的定义在 vtol_att_control_main.h 中
* 进行各种变量的初始化
*/
VtolAttitudeControl::VtolAttitudeControl() :
	_task_should_exit(false), //二者配合，为了线程的运行安全
	_control_task(-1),

	// mavlink log
	// 对mavlink的记录消息进行显示
	_mavlink_log_pub(nullptr),

	// init subscription handlers
	// 初始化订阅句柄
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_mc_virtual_att_sp_sub(-1),
	_fw_virtual_att_sp_sub(-1),
	_mc_virtual_v_rates_sp_sub(-1),
	_fw_virtual_v_rates_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_local_pos_sub(-1),
	_airspeed_sub(-1),
	_battery_status_sub(-1),
	_vehicle_cmd_sub(-1),
	_tecs_status_sub(-1),
	_land_detected_sub(-1),

	// init publication handlers
	// 初始化发布句柄
	_actuators_0_pub(nullptr),
	_actuators_1_pub(nullptr),
	_vtol_vehicle_status_pub(nullptr),
	_v_rates_sp_pub(nullptr),
	_v_att_sp_pub(nullptr),

	// 初始化变量
	_transition_command(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC), //命令：处于多轴模式
	_abort_front_transition(false)

{
	// 初始化存储变量
	memset(& _vtol_vehicle_status, 0, sizeof(_vtol_vehicle_status)); //！！！ vtol 飞行器状态
	_vtol_vehicle_status.vtol_in_rw_mode = true;	/* start vtol in rotary wing mode*/
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_mc_virtual_att_sp, 0, sizeof(_mc_virtual_att_sp));
	memset(&_fw_virtual_att_sp, 0, sizeof(_fw_virtual_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_mc_virtual_v_rates_sp, 0, sizeof(_mc_virtual_v_rates_sp));
	memset(&_fw_virtual_v_rates_sp, 0, sizeof(_fw_virtual_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators_out_0, 0, sizeof(_actuators_out_0));
	memset(&_actuators_out_1, 0, sizeof(_actuators_out_1));
	memset(&_actuators_mc_in, 0, sizeof(_actuators_mc_in));
	memset(&_actuators_fw_in, 0, sizeof(_actuators_fw_in));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_local_pos, 0, sizeof(_local_pos));
	memset(&_airspeed, 0, sizeof(_airspeed));
	memset(&_batt_status, 0, sizeof(_batt_status));
	memset(&_vehicle_cmd, 0, sizeof(_vehicle_cmd));
	memset(&_tecs_status, 0, sizeof(_tecs_status));
	memset(&_land_detected, 0, sizeof(_land_detected));

	_params.idle_pwm_mc = PWM_DEFAULT_MIN;
	_params.vtol_motor_count = 0;
	_params.vtol_fw_permanent_stab = 0;

	// 不同的参数对应不同的句柄
	_params_handles.idle_pwm_mc = param_find("VT_IDLE_PWM_MC");
	_params_handles.vtol_motor_count = param_find("VT_MOT_COUNT");
	_params_handles.vtol_fw_permanent_stab = param_find("VT_FW_PERM_STAB");
	_params_handles.mc_airspeed_min = param_find("VT_MC_ARSPD_MIN");
	_params_handles.mc_airspeed_max = param_find("VT_MC_ARSPD_MAX");
	_params_handles.mc_airspeed_trim = param_find("VT_MC_ARSPD_TRIM");
	_params_handles.fw_pitch_trim = param_find("VT_FW_PITCH_TRIM");
	_params_handles.power_max = param_find("VT_POWER_MAX");
	_params_handles.prop_eff = param_find("VT_PROP_EFF");
	_params_handles.arsp_lp_gain = param_find("VT_ARSP_LP_GAIN");
	_params_handles.vtol_type = param_find("VT_TYPE");
	_params_handles.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK");
	_params_handles.fw_min_alt = param_find("VT_FW_MIN_ALT");

	/* fetch initial parameter values */
	// 根据句柄对必要的控制参数进行更新
	parameters_update();

	// 根据不同的飞行器类型，建立不同的类型变量
	if (_params.vtol_type == vtol_type::TAILSITTER) {
		_vtol_type = new Tailsitter(this);
	} else if (_params.vtol_type == vtol_type::TILTROTOR) { //！！！
		_vtol_type = new Tiltrotor(this);
	} else if (_params.vtol_type == vtol_type::STANDARD) {
		_vtol_type = new Standard(this);
	} else {
		_task_should_exit = true;
	}
}

/**
* Destructor
*/
VtolAttitudeControl::~VtolAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true; // 这个会导致直接跳出task_main中的打循环

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
		} while (_control_task != -1);
	}

	// free memory used by instances of base class VtolType
	if (_vtol_type != nullptr) {
		delete _vtol_type;
	}

	VTOL_att_control::g_control = nullptr;
}

/* ↓ ********* 各种更新订阅的变量 ********* ↓ */

/**
* Check for changes in vehicle control mode.
* 读取订阅
*/
void VtolAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

/**
* Check for changes in manual inputs.
* 读取订阅
*/
void VtolAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}
/**
* Check for arming status updates.
* 读取订阅
*/
void VtolAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

/**
* Check for inputs from mc attitude controller.
* 读取订阅，多轴姿态控制量
*/
void VtolAttitudeControl::actuator_controls_mc_poll()
{
	bool updated;
	orb_check(_actuator_inputs_mc, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_mc), _actuator_inputs_mc , &_actuators_mc_in);
	}
}

/**
* Check for inputs from fw attitude controller.
* 读取订阅，固定翼姿态控制量
*/
void VtolAttitudeControl::actuator_controls_fw_poll()
{
	bool updated;
	orb_check(_actuator_inputs_fw, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_fw), _actuator_inputs_fw , &_actuators_fw_in);
	}
}

/**
* Check for attitude rates setpoint from mc attitude controller
* 读取订阅，姿态速度期望，姿态环的输出（多轴）
*/
void VtolAttitudeControl::vehicle_rates_sp_mc_poll()
{
	bool updated;
	orb_check(_mc_virtual_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(mc_virtual_rates_setpoint), _mc_virtual_v_rates_sp_sub , &_mc_virtual_v_rates_sp);
	}
}

/**
* Check for attitude rates setpoint from fw attitude controller
* 读取订阅，姿态速度期望，姿态环的输出（固定翼）
*/
void VtolAttitudeControl::vehicle_rates_sp_fw_poll()
{
	bool updated;
	orb_check(_fw_virtual_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(fw_virtual_rates_setpoint), _fw_virtual_v_rates_sp_sub , &_fw_virtual_v_rates_sp);
	}
}

/**
* Check for airspeed updates.
* 读取订阅
*/
void
VtolAttitudeControl::vehicle_airspeed_poll()
{
	bool updated;
	orb_check(_airspeed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub , &_airspeed);
	}
}

/**
* Check for attitude set points update.
* 读取订阅，姿态期望值
*/
void
VtolAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

/**
* Check for attitude update.
* 读取订阅
*/
void
VtolAttitudeControl::vehicle_attitude_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

/**
* Check for battery updates.
* 读取订阅
*/
void
VtolAttitudeControl::vehicle_battery_poll()
{
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub , &_batt_status);
	}
}

/**
* Check for parameter updates.
* 读取订阅，用于更新控制参数
*/
void
VtolAttitudeControl::parameters_update_poll()
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

/**
* Check for sensor updates.
* 读取订阅
*/
void
VtolAttitudeControl::vehicle_local_pos_poll()
{
	bool updated;
	/* Check if parameters have changed */
	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub , &_local_pos);
	}

}

/**
* Check for mc virtual attitude setpoint updates.
* 读取订阅，姿态期望值，速度环的输出（多轴）
*/
void
VtolAttitudeControl::mc_virtual_att_sp_poll()
{
	bool updated;

	orb_check(_mc_virtual_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(mc_virtual_attitude_setpoint), _mc_virtual_att_sp_sub , &_mc_virtual_att_sp);
		/*PX4_INFO("New mc_virtual_attitude_setpoint: %.3f, %.3f, %.3f",
				(double)_mc_virtual_att_sp.roll_body,
				(double)_mc_virtual_att_sp.pitch_body,
				(double)_mc_virtual_att_sp.yaw_body);*/
	}

}

/**
* Check for fw virtual attitude setpoint updates.
* 读取订阅，姿态期望值，速度环的输出（固定翼）
*/
void
VtolAttitudeControl::fw_virtual_att_sp_poll()
{
	bool updated;

	orb_check(_fw_virtual_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(fw_virtual_attitude_setpoint), _fw_virtual_att_sp_sub , &_fw_virtual_att_sp);
	}

}

/**
* Check for command updates.
* 读取订阅，处理一个“命令”
*/
void
VtolAttitudeControl::vehicle_cmd_poll()
{
	bool updated;
	orb_check(_vehicle_cmd_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_command), _vehicle_cmd_sub , &_vehicle_cmd);
		handle_command();
	}
}

/**
* Check for TECS status updates.
* 读取订阅
*/
void
VtolAttitudeControl::tecs_status_poll()
{
	bool updated;

	orb_check(_tecs_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(tecs_status), _tecs_status_sub , &_tecs_status);
	}
}

/**
* Check for land detector updates.
* 读取订阅
*/
void
VtolAttitudeControl::land_detected_poll()
{
	bool updated;

	orb_check(_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _land_detected_sub , &_land_detected);
	}
}

/* ↑ ********* 各种更新订阅的变量 ********* ↑ */

/**
* Check received command
* 检测是否需要根据命令进行倾转
*/
void
VtolAttitudeControl::handle_command()
{
	// update transition command if necessary
	// 一个topic指令，应该是mavlink消息传过来的
	if (_vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
		_transition_command = int(_vehicle_cmd.param1 + 0.5f);
	}
}

// 是否向固定翼模式切换，通过 切换开关 or 命令
/*
 * Returns true if fixed-wing mode is requested.
 * Changed either via switch or via command.
 */
bool
VtolAttitudeControl::is_fixed_wing_requested()
{
	bool to_fw = false;

	if (_manual_control_sp.transition_switch != manual_control_setpoint_s::SWITCH_POS_NONE &&
	    _v_control_mode.flag_control_manual_enabled) { // 手动下，如果过渡开关被扳动
		to_fw = (_manual_control_sp.transition_switch == manual_control_setpoint_s::SWITCH_POS_ON);
		// 如果开关完全打开，表明要向固定翼模式切换
	} else {
		// listen to transition commands if not in manual or mode switch is not mapped
		// 如果没有过渡开关的映射，或者不是在手动控制下，根据命令进行倾转
		to_fw = (_transition_command == vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW); //来着 handle_command()
	}

	// handle abort request
	// 如果存在终止倾转的设置
	if (_abort_front_transition) {
		if (to_fw) {
			to_fw = false;
		} else {
			// the state changed to mc mode, reset the abort request
			_abort_front_transition = false;
			_vtol_vehicle_status.vtol_transition_failsafe = false;
		}
	}

	return to_fw;
}

/*
 * Abort front transition
 * 终止
 */
// 一般是因为飞行高度导致，在vtol_type中调用
void
VtolAttitudeControl::abort_front_transition(const char *reason)
{
	if (!_abort_front_transition) { //对前向倾转进行终止
		mavlink_log_critical(&_mavlink_log_pub, "Abort: %s", reason);
		_abort_front_transition = true;
		_vtol_vehicle_status.vtol_transition_failsafe = true; //失效保护
	}
}

/**
* Update parameters.
* 更新 vtol 类飞行器的通用参数，并且更新各种类型 vtol 飞行器的特有参数
*/
int
VtolAttitudeControl::parameters_update()
{
	float v;
	int l;
	/* idle pwm for mc mode */
	param_get(_params_handles.idle_pwm_mc, &_params.idle_pwm_mc);

	/* vtol motor count */
	param_get(_params_handles.vtol_motor_count, &_params.vtol_motor_count);

	/* vtol fw permanent stabilization */
	param_get(_params_handles.vtol_fw_permanent_stab, &_params.vtol_fw_permanent_stab);

	/* vtol mc mode min airspeed */
	param_get(_params_handles.mc_airspeed_min, &v);
	_params.mc_airspeed_min = v;

	/* vtol mc mode max airspeed */
	param_get(_params_handles.mc_airspeed_max, &v);
	_params.mc_airspeed_max = v;

	/* vtol mc mode trim airspeed */
	param_get(_params_handles.mc_airspeed_trim, &v);
	_params.mc_airspeed_trim = v;

	/* vtol pitch trim for fw mode */
	param_get(_params_handles.fw_pitch_trim, &v);
	_params.fw_pitch_trim = v;

	/* vtol maximum power engine can produce */
	param_get(_params_handles.power_max, &v);
	_params.power_max = v;

	/* vtol propeller efficiency factor */
	param_get(_params_handles.prop_eff, &v);
	_params.prop_eff = v;

	/* vtol total airspeed estimate low pass gain */
	param_get(_params_handles.arsp_lp_gain, &v);
	_params.arsp_lp_gain = v;

	param_get(_params_handles.vtol_type, &l);
	_params.vtol_type = l;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles.elevons_mc_lock, &l);
	_params.elevons_mc_lock = l;

	/* minimum relative altitude for FW mode (QuadChute) */
	param_get(_params_handles.fw_min_alt, &v);
	_params.fw_min_alt = v;


	// update the parameters of the instances of base VtolType
	if (_vtol_type != nullptr) {
		_vtol_type->parameters_update();
	}

	return OK;
}

/**
* Prepare message for mc attitude rates setpoint topic
* 直接获得多轴的角速度期望值
*/
void VtolAttitudeControl::fill_mc_att_rates_sp()
{
	_v_rates_sp.timestamp 	= _mc_virtual_v_rates_sp.timestamp;
	_v_rates_sp.roll 	= _mc_virtual_v_rates_sp.roll;
	_v_rates_sp.pitch 	= _mc_virtual_v_rates_sp.pitch;
	_v_rates_sp.yaw 	= _mc_virtual_v_rates_sp.yaw;
	_v_rates_sp.thrust 	= _mc_virtual_v_rates_sp.thrust;
}
/**
* Prepare message for fw attitude rates setpoint topic
* 直接获得固定翼的角速度期望值
*/
void VtolAttitudeControl::fill_fw_att_rates_sp()
{
	_v_rates_sp.timestamp 	= _fw_virtual_v_rates_sp.timestamp;
	_v_rates_sp.roll 	= _fw_virtual_v_rates_sp.roll;
	_v_rates_sp.pitch 	= _fw_virtual_v_rates_sp.pitch;
	_v_rates_sp.yaw 	= _fw_virtual_v_rates_sp.yaw;
	_v_rates_sp.thrust 	= _fw_virtual_v_rates_sp.thrust;
}

// 发布角度期望值
// _v_att_sp的值由Tiltrotor::update_mc_state()或Tiltrotor::update_fw_state()或Tiltrotor::update_transition_state()更新
void VtolAttitudeControl::publish_att_sp()
{
	if (_v_att_sp_pub != nullptr) {
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_pub, &_v_att_sp);
	} else {
		/* advertise and publish */
		_v_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
	}
}

// 仅仅中转
void
VtolAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	VTOL_att_control::g_control->task_main();
}

/********** ！！！！！！！！！！！！！ **********/
/********** vtol 控制功能由此函数实现 **********/
/*
增益调度控制过程——
	1. 控制循环中主要关注的是来自多轴、固定翼的控制量actuator_controls_virtual_mc、actuator_controls_virtual_fw
	2. 每次循环中都会发布vtol的飞行状态topic vtol_vehicle_status
	3. 更新控制参数和其他的状态
	4. 利用_vtol_type->update_vtol_state()更新目前的倾转状态
	5. 利用_vtol_type->get_mode()获得倾转状态，不同状态下进行不同的权重更新、姿态期望、姿态角速度期望更新，并更新vtol飞行状态存储变量
	   过渡模式下的姿态期望、姿态角速度期望都取的多旋翼控制器的生产量，实际控制中的拉力控制量也是取得多旋翼控制器的生产量
	6. 发布姿态期望
	7. 根据计算的权重，生成vtol控制量，并发布给主通道和辅助通道
	8. 发布姿态角速度期望
*/
void VtolAttitudeControl::task_main()
{
	fflush(stdout);

	/* do subscriptions */
	// 订阅句柄赋值
	_v_att_sp_sub          = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_mc_virtual_att_sp_sub = orb_subscribe(ORB_ID(mc_virtual_attitude_setpoint));
	_fw_virtual_att_sp_sub = orb_subscribe(ORB_ID(fw_virtual_attitude_setpoint));
	_mc_virtual_v_rates_sp_sub = orb_subscribe(ORB_ID(mc_virtual_rates_setpoint));
	_fw_virtual_v_rates_sp_sub = orb_subscribe(ORB_ID(fw_virtual_rates_setpoint));
	_v_att_sub             = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_att_sp_sub          = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_control_mode_sub    = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub            = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub             = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub         = orb_subscribe(ORB_ID(vehicle_local_position));
	_airspeed_sub          = orb_subscribe(ORB_ID(airspeed));
	_battery_status_sub	   = orb_subscribe(ORB_ID(battery_status));
	_vehicle_cmd_sub	   = orb_subscribe(ORB_ID(vehicle_command));
	_tecs_status_sub = orb_subscribe(ORB_ID(tecs_status));
	_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	// 多轴与固定翼控制器生成的名义控制量
	_actuator_inputs_mc    = orb_subscribe(ORB_ID(actuator_controls_virtual_mc));
	_actuator_inputs_fw    = orb_subscribe(ORB_ID(actuator_controls_virtual_fw));

	// 必要控制参数更新
	parameters_update();  // initialize parameter cache

	/* update vtol vehicle status*/
	_vtol_vehicle_status.fw_permanent_stab = _params.vtol_fw_permanent_stab == 1 ? true : false;

	// make sure we start with idle in mc mode
	_vtol_type->set_idle_mc();

	/* wakeup source*/
	px4_pollfd_struct_t fds[3] = {};	/*input_mc, input_fw, parameters*/

	fds[0].fd     = _actuator_inputs_mc;
	fds[0].events = POLLIN; //更新
	fds[1].fd     = _actuator_inputs_fw;
	fds[1].events = POLLIN;
	fds[2].fd     = _params_sub; //控制参数订阅句柄
	fds[2].events = POLLIN;

	while (!_task_should_exit) {
		/*Advertise/Publish vtol vehicle status*/
		_vtol_vehicle_status.timestamp = hrt_absolute_time();
		// 每次循环都发布一次 vtol 飞行器的状态
		if (_vtol_vehicle_status_pub != nullptr) {
			orb_publish(ORB_ID(vtol_vehicle_status), _vtol_vehicle_status_pub, &_vtol_vehicle_status);

		} else {
			_vtol_vehicle_status_pub = orb_advertise(ORB_ID(vtol_vehicle_status), &_vtol_vehicle_status);
		}

		/* wait for up to 100ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
		/* timed out - periodic check for _task_should_exit */
		//如果 timeout 了，进行周期检测
		if (pret == 0) {
			continue;
		}
		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[2].revents & POLLIN) {	//parameters were updated, read them now
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);
			/* update parameters from storage */
			parameters_update();
		}

		// 固定翼模式下是否手动也自动稳定高度
		_vtol_vehicle_status.fw_permanent_stab = _params.vtol_fw_permanent_stab == 1 ? true : false;

		// 更新各种订阅了的变量
		mc_virtual_att_sp_poll();
		fw_virtual_att_sp_poll();
		vehicle_control_mode_poll();	//Check for changes in vehicle control mode.
		vehicle_manual_poll();			//Check for changes in manual inputs.
		arming_status_poll();			//Check for arming status updates.
		vehicle_attitude_setpoint_poll();//Check for changes in attitude set points
		vehicle_attitude_poll();		//Check for changes in attitude
		actuator_controls_mc_poll();	//Check for changes in mc_attitude_control output
		actuator_controls_fw_poll();	//Check for changes in fw_attitude_control output
		vehicle_rates_sp_mc_poll();
		vehicle_rates_sp_fw_poll();
		parameters_update_poll();
		vehicle_local_pos_poll();			// Check for new sensor values
		vehicle_airspeed_poll();
		vehicle_battery_poll();
		vehicle_cmd_poll();
		tecs_status_poll();
		land_detected_poll();

		// update the vtol state machine which decides which mode we are in
		// ！！！
		_vtol_type->update_vtol_state(); //更新状态机状态 _vtol_schedule

		// reset transition command if not auto control
		// 处于手动下，根据实际状态使倾转命令保持一致
		if (_v_control_mode.flag_control_manual_enabled) {
			if (_vtol_type->get_mode() == ROTARY_WING) {
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;

			} else if (_vtol_type->get_mode() == FIXED_WING) {
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW;

			} else if (_vtol_type->get_mode() == TRANSITION_TO_MC) {
				/* We want to make sure that a mode change (manual>auto) during the back transition
				 * doesn't result in an unsafe state. This prevents the instant fall back to
				 * fixed-wing on the switch from manual to auto */
				// 避免失速的危险
				_transition_command = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
			}
		}

		// 根据当前的飞行模式，采用不同的状态更新
		// check in which mode we are in and call mode specific functions
		if (_vtol_type->get_mode() == ROTARY_WING) { //旋翼模式下
			// vehicle is in rotary wing mode
			_vtol_vehicle_status.vtol_in_rw_mode = true;
			_vtol_vehicle_status.vtol_in_trans_mode = false;

			// got data from mc attitude controller
			if (fds[0].revents & POLLIN) {
				// 获得旋翼控制器的控制量
				orb_copy(ORB_ID(actuator_controls_virtual_mc), _actuator_inputs_mc, &_actuators_mc_in);
				// 获得多轴角度期望值
				_vtol_type->update_mc_state();
				// 获得多轴角速度期望值
				fill_mc_att_rates_sp();
			}

		} else if (_vtol_type->get_mode() == FIXED_WING) { //固定翼模式下
			// vehicle is in fw mode
			_vtol_vehicle_status.vtol_in_rw_mode = false;
			_vtol_vehicle_status.vtol_in_trans_mode = false;

			// got data from fw attitude controller
			if (fds[1].revents & POLLIN) {
				// 获得固定翼控制器的控制量
				orb_copy(ORB_ID(actuator_controls_virtual_fw), _actuator_inputs_fw, &_actuators_fw_in);
				// 人的控制输入
				vehicle_manual_poll();
				// 获得固定翼角度期望值
				_vtol_type->update_fw_state();
				// 获得固定翼角速度期望值
				fill_fw_att_rates_sp();
			}

		} else if (_vtol_type->get_mode() == TRANSITION_TO_MC || _vtol_type->get_mode() == TRANSITION_TO_FW) {
			// vehicle is doing a transition
			_vtol_vehicle_status.vtol_in_trans_mode = true;
			_vtol_vehicle_status.vtol_in_rw_mode = true; //making mc attitude controller work during transition
			                                             //要用多轴里面的定高控制
			_vtol_vehicle_status.in_transition_to_fw = (_vtol_type->get_mode() == TRANSITION_TO_FW); //是向固定翼过渡还是像多轴过渡

			bool got_new_data = false;

			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_virtual_mc), _actuator_inputs_mc, &_actuators_mc_in);
				got_new_data = true;
			}

			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_virtual_fw), _actuator_inputs_fw, &_actuators_fw_in);
				got_new_data = true;
			}

			// update transition state if got any new data
			if (got_new_data) {
				// 获得“多轴”的角度期望，目的时为了实现定高
				_vtol_type->update_transition_state();
				// 获得“多轴”的角速度期望
				fill_mc_att_rates_sp();
				publish_att_sp();
			}

		} else if (_vtol_type->get_mode() == EXTERNAL) {
			// we are using external module to generate attitude/thrust setpoint
			// 似乎并没有什么用
			_vtol_type->update_external_state();
		}

		publish_att_sp(); //发布姿态期望
		// ！！！
		// vtol 的控制量
		_vtol_type->fill_actuator_outputs();
		// 将控制量发布出去
		/* Only publish if the proper mode(s) are enabled */
		if (_v_control_mode.flag_control_attitude_enabled ||
		    _v_control_mode.flag_control_rates_enabled ||
		    _v_control_mode.flag_control_manual_enabled) {
			if (_actuators_0_pub != nullptr) { //主控制通道的控制量
				orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators_out_0);
			} else {
				_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators_out_0);
			}

			if (_actuators_1_pub != nullptr) { //辅助控制通道的控制量
				orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_out_1);

			} else {
				_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_out_1);
			}
		}

		// 发布姿态角速度期望，要么是多轴的，要么是固定翼的
		if (_v_rates_sp_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_pub, &_v_rates_sp);

		} else {
			_v_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_v_rates_sp);
		}
		// vtol的控制仅仅处于姿态环，所以这里发布的姿态期望，实际上就是遥控器设定的
		// 而姿态角速度期望由于采用的控制器不同，要么发布多轴的，要么发布固定翼的
	}

	warnx("exit");
	_control_task = -1;
	return;
}

int
VtolAttitudeControl::start()
{
	ASSERT(_control_task == -1); //声称

	/* start the task */
	_control_task = px4_task_spawn_cmd("vtol_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 10,
					   1200,
					   (px4_main_t)&VtolAttitudeControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

/********** 打开此线程 **********/
int vtol_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: vtol_att_control {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (VTOL_att_control::g_control != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		// 建立一个新的 class，首先会运行“构造函数” VtolAttitudeControl::VtolAttitudeControl
		VTOL_att_control::g_control = new VtolAttitudeControl;

		if (VTOL_att_control::g_control == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		// 指向 class 中的另一个子函数 VtolAttitudeControl::start()
		if (OK != VTOL_att_control::g_control->start()) {
			delete VTOL_att_control::g_control;
			VTOL_att_control::g_control = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (VTOL_att_control::g_control == nullptr) {
			PX4_WARN("not running");
			return 0;
		}

		// 删除一个class，会运行析构函数
		delete VTOL_att_control::g_control;
		VTOL_att_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (VTOL_att_control::g_control) {
			PX4_WARN("running");

		} else {
			PX4_WARN("not running");
		}

		return 0;
	}

	PX4_WARN("unrecognized command");
	return 1;
}
