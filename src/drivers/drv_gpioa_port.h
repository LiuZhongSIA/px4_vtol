/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file drv_gpioa_port.h
 *
 * gpioa_port driver API
 */

#pragma once

#include <stdint.h>
#include <sys/ioctl.h>

#define GPIOA_PORT_DEVICE_PATH		"/dev/tone_alarm0"

#define _GPIOA_PORT_BASE	0x7400

#define GPIOA_PORT_ON			_IOC(_GPIOA_PORT_BASE, 0)
#define GPIOA_PORT_OFF			_IOC(_GPIOA_PORT_BASE, 1)
#define GPIOA_PORT_TOGGLE		_IOC(_GPIOA_PORT_BASE, 2)

__BEGIN_DECLS

/*
 * Initialise the LED driver.
 */
__EXPORT void gpioa_port_main(void);
__EXPORT void gpioa_port_init(void);
__EXPORT void gpioa_port_on(void);
__EXPORT void gpioa_port_off(void);
__EXPORT void gpioa_port_toggle(void);

__END_DECLS
