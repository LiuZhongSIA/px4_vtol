/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
* @file vtol_type.cpp
*
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
*
*/
// Read 3

#include "vtol_type.h"
#include "drivers/drv_pwm_output.h"
#include <px4_defines.h>
#include "vtol_att_control_main.h"
// 注意: Tiltrotor 是 vtol_type 的继承
//      其他结构飞行器的class也是vtol_type 的继承

// 构造函数
VtolType::VtolType(VtolAttitudeControl *att_controller) :
	_attc(att_controller),
	_vtol_mode(ROTARY_WING)
{
	_v_att = _attc->get_att();
	_v_att_sp = _attc->get_att_sp();
	_mc_virtual_att_sp = _attc->get_mc_virtual_att_sp();
	_fw_virtual_att_sp = _attc->get_fw_virtual_att_sp();
	_v_rates_sp = _attc->get_rates_sp();
	_mc_virtual_v_rates_sp = _attc->get_mc_virtual_rates_sp();
	_fw_virtual_v_rates_sp = _attc->get_fw_virtual_rates_sp();
	_manual_control_sp = _attc->get_manual_control_sp();
	_v_control_mode = _attc->get_control_mode();
	_vtol_vehicle_status = _attc->get_vtol_vehicle_status();
	_actuators_out_0 = _attc->get_actuators_out0();
	_actuators_out_1 = _attc->get_actuators_out1();
	_actuators_mc_in = _attc->get_actuators_mc_in();
	_actuators_fw_in = _attc->get_actuators_fw_in();
	_armed = _attc->get_armed();
	_local_pos = _attc->get_local_pos();
	_airspeed = _attc->get_airspeed();
	_batt_status = _attc->get_batt_status();
	_tecs_status = _attc->get_tecs_status();
	_land_detected = _attc->get_land_detected();
	_params = _attc->get_params(); // vtol 的通用参数

	flag_idle_mc = true; // 为多轴模式设置闲置的旋翼转速
}

VtolType::~VtolType()
{

}

/**
* Adjust idle speed for mc mode.
*/
// vtol 的通用函数，为多轴模式设置闲置旋翼转速
void VtolType::set_idle_mc()
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH; //设备名称：/dev/pwm_output0
	int fd = px4_open(dev, 0); //打开设备
	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	// 要写入PWM口的数量？
	// 但是并没有赋值
	unsigned servo_count;
	int ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	unsigned pwm_value = _params->idle_pwm_mc; //读取设置的参数
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));
	for (int i = 0; i < _params->vtol_motor_count; i++) {
		pwm_values.values[i] = pwm_value; //赋值
		pwm_values.channel_count++;
	}
	// 将设置的参数写入
	ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);
	if (ret != OK) {
		PX4_WARN("failed setting min values");
	}

	px4_close(fd);
	flag_idle_mc = true;
}

/**
* Adjust idle speed for fw mode.
*/
// vtol 的通用函数，为固定翼模式设置闲置旋翼转速
void VtolType::set_idle_fw()
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	int fd = px4_open(dev, 0);
	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));
	for (int i = 0; i < _params->vtol_motor_count; i++) {
		pwm_values.values[i] = PWM_MOTOR_OFF; //固定翼模式下电机是不转的
		pwm_values.channel_count++;
	}
	// 将设置的参数写入
	int ret = px4_ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);
	if (ret != OK) {
		PX4_WARN("failed setting min values");
	}

	px4_close(fd);
}

// Tiltrotor 中会调用此函数功能
void VtolType::update_mc_state()
{
	// copy virtual attitude setpoint to real attitude setpoint
	// 读取多轴姿态期望值
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;
}

// Tiltrotor 中会调用此函数功能
void VtolType::update_fw_state()
{
	// copy virtual attitude setpoint to real attitude setpoint
	// 读取固定翼姿态期望值
	memcpy(_v_att_sp, _fw_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
	_mc_roll_weight = 0.0f;
	_mc_pitch_weight = 0.0f;
	_mc_yaw_weight = 0.0f;

	// TECS——Total Energy Control System
	// 固定翼飞行器的控制方法，输出的拉力和俯仰角用于实现期望的下沉和爬升速度
	// tecs didn't publish an update yet after the transition
	// 完成过渡前，TECS不会使用
	if (_tecs_status->timestamp < _trans_finished_ts) { //并没有对 _trans_finished_ts 进行更新，该值一直为0
		_tecs_running = false;
	} else if (!_tecs_running) {
		_tecs_running = true;
		_tecs_running_ts = hrt_absolute_time();
	}
	// TECS didn't publish yet or the position controller didn't publish yet AFTER tecs
	// only wait on TECS we're in a mode where it is actually running
	// 当TECS没有使用时，或者没有最新的更新，且在定高模式时，保持高度
	if ((!_tecs_running || (_tecs_running && _fw_virtual_att_sp->timestamp <= _tecs_running_ts))
	    && _v_control_mode->flag_control_altitude_enabled) {

		waiting_on_tecs(); //在 tiltrotor 中有定义，保持多轴时的拉力
	}

	check_quadchute_condition(); //是否禁止前向倾转
}

// Tiltrotor 中“不会”调用此函数功能
void VtolType::update_transition_state()
{
	check_quadchute_condition();
}

// vtol 的通用函数，判断在地面上是否可以完全倾转
// 如果已经 armed 了，就不能完全倾转
bool VtolType::can_transition_on_ground()
{
	return !_armed->armed || _land_detected->landed;
}

// vtol 的通用函数，根据高度判断是否可以进行前向倾转
void VtolType::check_quadchute_condition()
{
	// fixed-wing minimum altitude, armed, !landed
	if (_params->fw_min_alt > FLT_EPSILON
	    && _armed->armed && !_land_detected->landed) {

		if (-(_local_pos->z) < _params->fw_min_alt) {
			_attc->abort_front_transition("Minimum altitude");
		}
	}
}
