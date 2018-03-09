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
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Roll time constant
 *
 * Reduce if the system is too twitchy, increase if the response is too slow and sluggish.
 *
 * @unit s
 * @min 0.15
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_TC, 0.2f);

/**
 * Pitch time constant
 *
 * Reduce if the system is too twitchy, increase if the response is too slow and sluggish.
 *
 * @unit s
 * @min 0.15
 * @max 0.25
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_TC, 0.2f);

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @max 8
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 6.5f);

/**
 * Roll rate P gain (MC)
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain (MC)
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.05f);

/**
 * Roll rate D gain (MC)
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward (MC)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);

/**
 * Pitch rate P gain (MC)
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain (MC)
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.05f);

/**
 * Pitch rate D gain (MC)
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward (MC)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 5
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_P, 2.8f);

/**
 * Yaw rate P gain (MC)
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain (MC)
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate D gain (MC)
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward (MC)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw feed forward
 *
 * Feed forward weight for manual yaw control. 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_FF, 0.5f);

/**
 * Max roll rate
 *
 * Limit for roll rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_MAX, 220.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * Max yaw rate
 *
 * A value of significantly over 120 degrees per second can already lead to mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX, 200.0f);

/**
 * Max yaw rate in auto mode
 *
 * Limit for yaw rate, has effect for large rotations in autonomous mode,
 * to avoid large control output and mixer saturation. A value of significantly
 * over 60 degrees per second can already lead to mixer saturation.
 * A value of 30 degrees / second is recommended to avoid very audible twitches.
 *
 * @unit deg/s
 * @min 0.0
 * @max 120.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRAUTO_MAX, 45.0f);

/**
 * Max acro roll rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_R_MAX, 360.0f);

/**
 * Max acro pitch rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_P_MAX, 360.0f);

/**
 * Max acro yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 5
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_Y_MAX, 360.0f);

/**
 * Threshold for Rattitude mode
 *
 * Manual input needed in order to override attitude control rate setpoints
 * and instead pass manual stick inputs as rate setpoints
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_RATT_TH, 1.0f);

/**
 * Threshold for Throttle PID Attenuation (TPA)
 *
 * Magnitude of throttle setpoint at which to begin attenuating roll/pitch P gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_TPA_BREAK, 1.0f);

/**
 * Slope for Throttle PID Attenuation (TPA)
 *
 * Rate at which to attenuate roll/pitch P gain
 * Attenuation factor is 1.0 when throttle magnitude is below the setpoint
 * Above the setpoint, the attenuation factor is (1 - slope*(abs(throttle)-breakpoint))
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_TPA_SLOPE, 1.0f);

/**
 * Whether to scale outputs by battery power level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);

/*************************************************************************************************/
/*************************************************************************************************/

/**
 * Roll rate P gain (FW)
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRA_P_FW, 0.05f);

/**
 * Roll rate I gain (FW)
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRA_I_FW, 0.01f);

/**
 * Roll rate D gain (FW)
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRA_D_FW, 0.0f);

/**
 * Roll rate feedforward (FW)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRA_FF_FW, 0.0f);

/**
 * Pitch rate P gain (FW)
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRA_P_FW, 0.08f);

/**
 * Pitch rate I gain (FW)
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRA_I_FW, 0.02f);

/**
 * Pitch rate D gain (FW)
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRA_D_FW, 0.0f);

/**
 * Pitch rate feedforward (FW)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRA_FF_FW, 0.0f);

/**
 * Yaw rate P gain (FW)
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRA_P_FW, 0.05f);

/**
 * Yaw rate I gain (FW)
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.001
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRA_I_FW, 0.0f);

/**
 * Yaw rate D gain (FW)
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRA_D_FW, 0.0f);

/**
 * Yaw rate feedforward (FW)
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRA_FF_FW, 0.0f);

/*************************************************************************************************/
/*************************************************************************************************/

/**
 * Position of tilt servo 1 in mc mode (up left)
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_1_MC, 0.0f);

/**
 * Position of tilt servo 1 in fw mode (up left)
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_1_FW, 1.0f);

/**
 * Position of tilt servo 2 in mc mode (up right)
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_2_MC, 0.0f);

/**
 * Position of tilt servo 2 in fw mode (up right)
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_2_FW, 1.0f);

/**
 * Position of tilt servo 3 in mc mode (low right)
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_3_MC, 0.0f);

/**
 * Position of tilt servo 3 in fw mode (low right)
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_3_FW, 1.0f);

/**
 * Position of tilt servo 4 in mc mode (low left)
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_4_MC, 0.0f);

/**
 * Position of tilt servo 4 in fw mode (low left)
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_4_FW, 1.0f);