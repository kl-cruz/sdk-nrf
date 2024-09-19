/*
 * Copyright (c) 2024, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#define DT_DRV_COMPAT nrf_leds

/**
 * @file
 * @brief LEDs
 */

#include <soc.h>
#include <zephyr/drivers/led.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/pm/device.h>
#include <zephyr/kernel.h>

#include <helpers/nrfx_gppi.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(led_nrf, CONFIG_LED_NRF_LOG_LEVEL);

static int led_nrf_set_brightness(const struct device *dev,
				  uint32_t led, uint8_t value)
{
	
	return 0;
}

static int led_nrf_on(const struct device *dev, uint32_t led)
{
	return led_nrf_set_brightness(dev, led, 100);
}

static int led_nrf_off(const struct device *dev, uint32_t led)
{
	return led_nrf_set_brightness(dev, led, 0);
}

static int led_nrf_init(const struct device *dev)
{
	
	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int led_nrf_pm_action(const struct device *dev,
			     enum pm_device_action action)
{
	uint32_t gpio_nrf = *((uint32_t *)dev->config);

	if (action == PM_DEVICE_ACTION_SUSPEND)
	{
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct led_driver_api led_nrf_api = {
	.on		= led_nrf_on,
	.off		= led_nrf_off,
	.set_brightness	= led_nrf_set_brightness,
};

#define LED_NRF_DEVICE(id)					\
								\
PM_DEVICE_DT_INST_DEFINE(id, led_nrf_pm_action);		\
\
static uint32_t led_nrf_gpio##id = NRF_DT_GPIOS_TO_PSEL(DT_DRV_INST(id), gpios); \
								\
DEVICE_DT_INST_DEFINE(id, &led_nrf_init,			\
		      PM_DEVICE_DT_INST_GET(id), NULL,		\
		      &led_nrf_gpio##id, POST_KERNEL,	\
		      CONFIG_LED_NRF_INIT_PRIORITY, &led_nrf_api);

DT_INST_FOREACH_STATUS_OKAY(LED_NRF_DEVICE)
