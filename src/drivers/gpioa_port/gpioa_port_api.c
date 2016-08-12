#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>

#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_gpioa_port.h>
#include <drivers/boards/px4fmu-v2/board_config.h>
#include "gpioa_port_api.h"

//static int gpioa_ports = -1;

__EXPORT void gpioa_port_init()
{
	/* configure the GPIO to the idle state */
	stm32_configgpio(GPIOA_PORT_OUTPUT);
	/* first open normal GPIOA_PORTs */
	//gpioa_ports = open(GPIOA_PORT_DEVICE_PATH, 0);

//	if (gpioa_ports < 0) {
//		warnx("GPIOA_PORT: open fail\n");
//		return ERROR;
//	}
	//return 0;
}

__EXPORT void gpioa_port_deinit()
{
	;
}

__EXPORT void gpioa_port_toggle(void)
{
	;
}

__EXPORT void gpioa_port_on(void)
{
	stm32_gpiowrite(GPIOA_PORT_OUTPUT, true);
}

__EXPORT void gpioa_port_off(void)
{
	stm32_gpiowrite(GPIOA_PORT_OUTPUT, false);
}



