/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file motion_delay_test.c
 * motion_delay application example for PX4 autopilot
 *
 * @author huang li long <huanglilongwk@outlook.com>
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

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/topics/att_pos_mocap.h>
#include <drivers/drv_gpioa_port.h>

static bool thread_should_exit = false;		/**< motion_delay exit flag */
static bool thread_running = false;			/**< motion_delay status flag */
static int motion_delay_task;				/**< Handle of motion_delay task / thread */

orb_advert_t	_mocap_pub = 0;
int				_mocap_sub;

/**
 * motion_delay management function.
 */
__EXPORT int motion_delay_test_main(int argc, char *argv[]);

/**
 * Mainloop of motion_delay.
 */
int motion_delay_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: motion_delay {start|stop|status} [-p <additional params>]\n\n");
}

int motion_delay_thread_main(int argc, char *argv[])
{

	thread_running = true;

	_mocap_sub = orb_subscribe(ORB_ID(att_pos_mocap));
	static uint64_t prev_off_time;
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _mocap_sub;
	fds[0].events = POLLIN;

	gpioa_port_init();
	gpioa_port_off();
	prev_off_time=hrt_absolute_time();
	while (!thread_should_exit) {
		struct att_pos_mocap_s mocap_data;
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		float dt = (hrt_absolute_time() - prev_off_time) / 1000000.0f;
		if(dt > 2.0f) // > 2 seconds
		{
			gpioa_port_on();
		}
		if (pret == 0) {
			continue;
		}
		if (pret < 0) {
			usleep(100000);
			continue;
		}

		orb_copy(ORB_ID(att_pos_mocap), _mocap_sub, &mocap_data);
		if(mocap_data.x < -1.0f)
		{
			gpioa_port_off();
			prev_off_time = hrt_absolute_time();
			uint64_t times = hrt_absolute_time();
			mocap_data.y = (times - mocap_data.timestamp_received) / 1000000.0f;
			if (_mocap_pub != 0) {
				orb_publish(ORB_ID(att_pos_mocap), _mocap_pub, &mocap_data);

			} else {
				_mocap_pub = orb_advertise(ORB_ID(att_pos_mocap), &mocap_data);
			}
		}
	}

	thread_running = false;

	return 0;
}

int motion_delay_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("motion_delay already running\n");
			/* this is not an error */
			return 0;
		}
		gpioa_port_off();
		thread_should_exit = false;
		motion_delay_task = px4_task_spawn_cmd("motion_delay",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 motion_delay_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		gpioa_port_on();
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}
