/**
 * file		: gpioa_port.cpp
 * Author	: huanglilong
 * time		: 2016/3/8
 * brief	: gpioa0 for general port use, and use tone_alarm port
 */

#include <nuttx/config.h>
#include <drivers/device/device.h>
#include <drivers/drv_gpioa_port.h>
#include <drivers/boards/px4fmu-v2/board_config.h>

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */

void gpioa_port_init(void)
{
	/* configure the GPIO to the idle state */
	stm32_configgpio(GPIOA_PORT_OUTPUT);
}

void gpioa_port_on(void)
{
	stm32_gpiowrite(GPIOA_PORT_OUTPUT, true);
}

void gpioa_port_off(void)
{
	stm32_gpiowrite(GPIOA_PORT_OUTPUT, false);
}

void gpioa_port_toggle(void)
{

}

class GPIOA_PORT : device::CDev
{
public:
	GPIOA_PORT();
	virtual ~GPIOA_PORT();

	virtual int		init();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);
};

GPIOA_PORT::GPIOA_PORT() :
	CDev("gpioa_port", GPIOA_PORT_DEVICE_PATH)
{
	// force immediate init/device registration
	init();
}

GPIOA_PORT::~GPIOA_PORT()
{
}

int
GPIOA_PORT::init()
{
	CDev::init();
	gpioa_port_init();

	return 0;
}

int
GPIOA_PORT::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int result = OK;

	switch (cmd) {
	case GPIOA_PORT_ON:
		gpioa_port_on();
		break;

	case GPIOA_PORT_OFF:
		gpioa_port_off();
		break;

	case GPIOA_PORT_TOGGLE:
		gpioa_port_toggle();
		break;


	default:
		result = CDev::ioctl(filp, cmd, arg);
	}
	return result;
}

namespace
{
GPIOA_PORT	*gGPIOA_PORT;
}

void
gpioa_port_main(void)
{
	if (gGPIOA_PORT == nullptr) {
		gGPIOA_PORT = new GPIOA_PORT;
		if (gGPIOA_PORT != nullptr)
			gGPIOA_PORT->init();
	}
}

