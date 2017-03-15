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
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

#define ARSP_YAW_CTRL_DISABLE 7.0f	// airspeed at which we stop controlling yaw during a front transition

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc),
	_rear_motors(ENABLED),
	_tilt_control(0.0f),
	_min_front_trans_dur(0.5f)
{
	_vtol_schedule.flight_mode = MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tiltrotor.front_trans_dur = param_find("VT_F_TRANS_DUR"); //在倾转的 Part1 过程停留的时间
	_params_handles_tiltrotor.back_trans_dur = param_find("VT_B_TRANS_DUR"); //后向倾转时间长度
	_params_handles_tiltrotor.tilt_mc = param_find("VT_TILT_MC"); //多轴模式下的倾转舵机的位置
	_params_handles_tiltrotor.tilt_transition = param_find("VT_TILT_TRANS"); //过渡模式下的倾转舵机的位置
	_params_handles_tiltrotor.tilt_fw = param_find("VT_TILT_FW"); //固定翼模式下的倾转舵机的位置
	_params_handles_tiltrotor.airspeed_trans = param_find("VT_ARSP_TRANS"); //由 Part1 加速后，向固定翼模式切换的速度
	_params_handles_tiltrotor.airspeed_blend_start = param_find("VT_ARSP_BLEND"); //开始混合使用两个控制器控制量的速度
	_params_handles_tiltrotor.elevons_mc_lock = param_find("VT_ELEV_MC_LOCK"); //多轴模式下升降舵是不是锁住
	_params_handles_tiltrotor.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR"); //在倾转的 Part2 过程停留的时间
	_params_handles_tiltrotor.fw_motors_off = param_find("VT_FW_MOT_OFFID"); //固定翼模式下需要关闭的通道ID
}

Tiltrotor::~Tiltrotor()
{

}

// 更新 vtol 通用数据的同时，会通过 _vtol_type 更新此特有数据
void
Tiltrotor::parameters_update()
{
	float v;
	int l;

	/* motors that must be turned off when in fixed wing mode */
	param_get(_params_handles_tiltrotor.fw_motors_off, &l);
	_params_tiltrotor.fw_motors_off = get_motor_off_channels(l); //需要被关闭同的 bitmap


	/* vtol duration of a front transition */
	param_get(_params_handles_tiltrotor.front_trans_dur, &v);
	_params_tiltrotor.front_trans_dur = math::constrain(v, 1.0f, 5.0f); //约束范围

	/* vtol duration of a back transition */
	param_get(_params_handles_tiltrotor.back_trans_dur, &v);
	_params_tiltrotor.back_trans_dur = math::constrain(v, 0.0f, 5.0f); //约束范围

	/* vtol tilt mechanism position in mc mode */
	param_get(_params_handles_tiltrotor.tilt_mc, &v);
	_params_tiltrotor.tilt_mc = v;

	/* vtol tilt mechanism position in transition mode */
	param_get(_params_handles_tiltrotor.tilt_transition, &v);
	_params_tiltrotor.tilt_transition = v;

	/* vtol tilt mechanism position in fw mode */
	param_get(_params_handles_tiltrotor.tilt_fw, &v);
	_params_tiltrotor.tilt_fw = v;

	/* vtol airspeed at which it is ok to switch to fw mode */
	param_get(_params_handles_tiltrotor.airspeed_trans, &v);
	_params_tiltrotor.airspeed_trans = v;

	/* vtol airspeed at which we start blending mc/fw controls */
	param_get(_params_handles_tiltrotor.airspeed_blend_start, &v);
	_params_tiltrotor.airspeed_blend_start = v;

	/* vtol lock elevons in multicopter */
	param_get(_params_handles_tiltrotor.elevons_mc_lock, &l);
	_params_tiltrotor.elevons_mc_lock = l;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tiltrotor.front_trans_dur_p2, &v);
	_params_tiltrotor.front_trans_dur_p2 = v;

	/* avoid parameters which will lead to zero division in the transition code */
	// 要大于0.5s
	_params_tiltrotor.front_trans_dur = math::max(_params_tiltrotor.front_trans_dur, _min_front_trans_dur);

	if (_params_tiltrotor.airspeed_trans < _params_tiltrotor.airspeed_blend_start + 1.0f) {
		_params_tiltrotor.airspeed_trans = _params_tiltrotor.airspeed_blend_start + 1.0f;
	} //先进入混合控制，才能向固定翼模式切换
}

// 返回需要被关闭通道的 bitmap
int Tiltrotor::get_motor_off_channels(int channels)
{
	int channel_bitmap = 0;

	int channel;

	for (int i = 0; i < _params->vtol_motor_count; ++i) {
		channel = channels % 10; //取余

		if (channel == 0) {
			break;
		}

		channel_bitmap |= 1 << (channel - 1); //channel_bitmap = channel_bitmap | (1 << (channel - 1))
		                                      //例如，输入为34，得到的应该是二进制 0001100 的形式
		channels = channels / 10; //取整
	}

	return channel_bitmap;
}

// ！！！
// 旋翼倾转的状态机
// 前向倾转：倾转一个角度进行加速，速度足够后完成全部倾转
// 后向倾转：直接倾转过来
void Tiltrotor::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (!_attc->is_fixed_wing_requested()) { //遥控器上的倾转开关和倾转命令都没有触发
		                                     //一定是在多轴模式，或者要往多轴模式过渡
		// plane is in multicopter mode
		switch (_vtol_schedule.flight_mode) {
		case MC_MODE:
			break;

		case FW_MODE: //固定翼 -> 多轴
			_vtol_schedule.flight_mode 	= TRANSITION_BACK; //进入后向倾转模式
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case TRANSITION_FRONT_P1: //Part1 -> 多轴
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE; //直接进入多轴
			break;

		case TRANSITION_FRONT_P2: //Part2 -> 多轴
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = MC_MODE;
			break;

		case TRANSITION_BACK:
			if (_tilt_control <= _params_tiltrotor.tilt_mc) { //根据舵机位置，判断是不是进入了多轴模式
				_vtol_schedule.flight_mode = MC_MODE;
			}

			break;
		}

	} else { //遥控器上的倾转开关或倾转命令已经触发
             //一定是在向固定翼模式过渡，或者已经在固定翼模式了
		switch (_vtol_schedule.flight_mode) {
		case MC_MODE: //多轴 -> 固定翼
			// initialise a front transition
			_vtol_schedule.flight_mode 	= TRANSITION_FRONT_P1; //先到 Part1，为了加速
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case FW_MODE:
			break;

		case TRANSITION_FRONT_P1: //Part1 -> 固定翼

			// check if we have reached airspeed to switch to fw mode
			// also allow switch if we are not armed for the sake of bench testing
			if (_airspeed->indicated_airspeed_m_s >= _params_tiltrotor.airspeed_trans || can_transition_on_ground()) {
			// 当速度足够，或者在地面时，就开始进入过渡的 Part2
				_vtol_schedule.flight_mode = TRANSITION_FRONT_P2;
				_vtol_schedule.transition_start = hrt_absolute_time();
			}

			break;

		case TRANSITION_FRONT_P2: //Part2 -> 固定翼

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _params_tiltrotor.tilt_fw) { //根据舵机位置，判断是不是进入了固定翼模式
				_vtol_schedule.flight_mode = FW_MODE;
				_tilt_control = _params_tiltrotor.tilt_fw;
				// 这里可以对 _trans_finished_ts 进行赋值，表示倾转完成的时间
			}

			break;

		case TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = FW_MODE;
			break;
		}
	}

	// map tiltrotor specific control phases to simple control modes
	// 一个更简单的过渡模式表述
	switch (_vtol_schedule.flight_mode) {
	case MC_MODE:
		_vtol_mode = ROTARY_WING;
		break;

	case FW_MODE:
		_vtol_mode = FIXED_WING;
		break;

	case TRANSITION_FRONT_P1:
	case TRANSITION_FRONT_P2:
		_vtol_mode = TRANSITION_TO_FW;
		break;

	case TRANSITION_BACK:
		_vtol_mode = TRANSITION_TO_MC;
		break;
	}
}

// vtol_att_control_main 中，如果处于多轴模式，以此函数更新状态
void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state(); //读取多轴姿态期望值，多轴姿态权重置1

	// make sure motors are not tilted
	// 倾转轴的位置设置
	_tilt_control = _params_tiltrotor.tilt_mc;

	// enable rear motors
	// 尾部的旋翼是需要使能的
	if (_rear_motors != ENABLED) {
		set_rear_motor_state(ENABLED);
	}

	// set idle speed for rotary wing mode
	// 设置旋翼的闲置转速
	if (!flag_idle_mc) {
		set_idle_mc();
		flag_idle_mc = true;
	}
}

// vtol_att_control_main 中，如果处于固定翼模式，以此函数更新状态
void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state(); //读取固定翼姿态期望值，多轴姿态权重置0，TECS是否使用，是否终止前向倾转

	// make sure motors are tilted forward
	// 倾转轴的位置设置
	_tilt_control = _params_tiltrotor.tilt_fw;

	// disable rear motors
	// 尾部的旋翼禁能
	if (_rear_motors != DISABLED) {
		set_rear_motor_state(DISABLED);
	}

	// adjust idle for fixed wing flight
	// 设置旋翼的闲置转速
	if (flag_idle_mc) {
		set_idle_fw();
		flag_idle_mc = false;
	}
}

// 当处于过渡模式时，使用此函数更新状态
void Tiltrotor::update_transition_state()
{
	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_flag_was_in_trans_mode = true;
	}

	if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P1) { //处于过渡过程的 Part1
		// for the first part of the transition the rear rotors are enabled
		if (_rear_motors != ENABLED) {
			set_rear_motor_state(ENABLED);
		}

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _params_tiltrotor.tilt_transition) { //由多轴模式到倾转至某一角度进行加速，是一个缓慢倾转的过程
			_tilt_control = _params_tiltrotor.tilt_mc +
					fabsf(_params_tiltrotor.tilt_transition - _params_tiltrotor.tilt_mc) * (float)hrt_elapsed_time(
						&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur * 1000000.0f);
		}

		// do blending of mc and fw controls
		// 滚转控制的权重，要么为0,要么为1
		if (_airspeed->indicated_airspeed_m_s >= _params_tiltrotor.airspeed_blend_start) {
			_mc_roll_weight = 0.0f;
		} else {
			// at low speeds give full weight to mc
			_mc_roll_weight = 1.0f;
		}

		// disable mc yaw control once the plane has picked up speed
		_mc_yaw_weight = 1.0f;

		if (_airspeed->indicated_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;
		}

		_thrust_transition = _mc_virtual_att_sp->thrust; //旋翼控制器的输出的拉力值

	} else if (_vtol_schedule.flight_mode == TRANSITION_FRONT_P2) { //处于过渡过程的 Part2
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = _params_tiltrotor.tilt_transition + //同样是一个缓慢的倾转过程，但是速度要快很多
				fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_transition) * (float)hrt_elapsed_time(
					&_vtol_schedule.transition_start) / (_params_tiltrotor.front_trans_dur_p2 * 1000000.0f);
		_mc_roll_weight = 0.0f;

		_thrust_transition = _mc_virtual_att_sp->thrust; //旋翼控制器的输出的拉力值

	} else if (_vtol_schedule.flight_mode == TRANSITION_BACK) { //处于反向过渡
		if (_rear_motors != IDLE) {
			set_rear_motor_state(IDLE);
		}

		if (!flag_idle_mc) { //为旋翼设置闲置转速，这样有利于转速增大，用于多轴模式下的控制
			set_idle_mc();
			flag_idle_mc = true;
		}

		// tilt rotors back
		if (_tilt_control > _params_tiltrotor.tilt_mc) { //缓慢的倾转过程
			_tilt_control = _params_tiltrotor.tilt_fw -
					fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_mc) * (float)hrt_elapsed_time(
						&_vtol_schedule.transition_start) / (_params_tiltrotor.back_trans_dur * 1000000.0f);
		}

		// set zero throttle for backtransition otherwise unwanted moments will be created
		// 反向倾转时，将旋翼控制量油门设置为0,避免一些不必要的力矩
		_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

		_mc_roll_weight = 0.0f;

	}

	// 对权滚转和航向权重进行限幅
	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);

	// copy virtual attitude setpoint to real attitude setpoint (we use multicopter att sp)
	// 读取多轴姿态期望值，为了保持拉力定高
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));
}

// 仅在 update_fw_state 中调用
void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust = _thrust_transition;
}

// ！！！
// 根据权重计算控制量
/**
* Write data to actuator output topic.
*/
void Tiltrotor::fill_actuator_outputs()
{
	// 主控制通道，控制电机
	_actuators_out_0->timestamp = _actuators_mc_in->timestamp;
	// 滚转、俯仰、航向、油门
	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
								_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight; //滚转的权重为 0 or 1
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
								_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight; //俯仰的权重在过渡模式下就没有变过，一直为1
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
								_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_yaw_weight; //航向的权重为 0 or 1
	if (_vtol_schedule.flight_mode == FW_MODE) { //油门的权重也是为 0 or 1
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
									_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];
	} else {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
									_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE];;
	}

	// 辅助制通道，控制舵机
	_actuators_out_1->timestamp = _actuators_fw_in->timestamp;
	// 滚转、俯仰、航向、倾转轴
	_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
								-_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL] * (1 - _mc_roll_weight);
	_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
								(_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH] + _params->fw_pitch_trim) * (1 - _mc_pitch_weight);
	_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
								_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * (1 - _mc_yaw_weight);
	_actuators_out_1->control[4] = _tilt_control;
}


/**
* Set state of rear motors.
*/
// 设置尾部螺旋桨的状态，在上面不同模式下的状态更新中用到
void Tiltrotor::set_rear_motor_state(rear_motor_state state)
{
	int pwm_value = PWM_DEFAULT_MAX;

	// map desired rear rotor state to max allowed pwm signal
	switch (state) {
	case ENABLED:
		pwm_value = PWM_DEFAULT_MAX; //最大的PWM输出
		_rear_motors = ENABLED;
		break;

	case DISABLED:
		pwm_value = PWM_MOTOR_OFF;
		_rear_motors = DISABLED;
		break;

	case IDLE:
		pwm_value = _params->idle_pwm_mc;
		_rear_motors = IDLE;
		break;
	}

	int ret;
	unsigned servo_count;
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;  //设备名称：/dev/pwm_output0
	int fd = px4_open(dev, 0); //打开设备

	if (fd < 0) {
		PX4_WARN("can't open %s", dev);
	}

	ret = px4_ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	struct pwm_output_values pwm_max_values;
	memset(&pwm_max_values, 0, sizeof(pwm_max_values));

	for (int i = 0; i < _params->vtol_motor_count; i++) {
		if (is_motor_off_channel(i)) {
			pwm_max_values.values[i] = pwm_value;

		} else {
			pwm_max_values.values[i] = PWM_DEFAULT_MAX;
		}

		pwm_max_values.channel_count = _params->vtol_motor_count;
	}

	ret = px4_ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_max_values);

	if (ret != OK) {PX4_WARN("failed setting max values");}

	px4_close(fd);
}

// 检测一个通道是否被关闭
bool Tiltrotor::is_motor_off_channel(const int channel)
{
	return (_params_tiltrotor.fw_motors_off >> channel) & 1;
}
