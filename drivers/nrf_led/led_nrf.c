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

static const nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(130);
static const nrfx_timer_t timer = NRFX_TIMER_INSTANCE(130);



static void timer_dummy_handler(nrf_timer_event_t event_type, void * p_context)
{

}

static int led_nrf_set_brightness(const struct device *dev,
				  uint32_t led, uint8_t value)
{
	uint32_t us_period = value * 100;

	if (value > 100) {
		return -EINVAL;
	}

	uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer, us_period);
    nrfx_timer_compare(&timer, NRF_TIMER_CC_CHANNEL1, desired_ticks, false);
	
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
	nrfx_err_t err;
	uint8_t out_channel;

	uint32_t gpio_nrf = *((uint32_t *)dev->config);

	err = nrfx_gpiote_init(&gpiote, 0);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_init error: 0x%08X", err);
		return 0;
	}

	err = nrfx_gpiote_channel_alloc(&gpiote, &out_channel);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to allocate out_channel, error: 0x%08X", err);
		return 0;
	}

	/* Initialize output pin. SET task will turn the LED on,
	 * CLR will turn it off and OUT will toggle it.
	 */
	static const nrfx_gpiote_output_config_t output_config = {
		.drive = NRF_GPIO_PIN_S0S1,
		.input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
		.pull = NRF_GPIO_PIN_NOPULL,
	};
	const nrfx_gpiote_task_config_t task_config = {
		.task_ch = out_channel,
		.polarity = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
	};

	err = nrfx_gpiote_output_configure(&gpiote, gpio_nrf,
					   &output_config,
					   &task_config);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_output_configure error: 0x%08X", err);
		return 0;
	}

	nrfx_gpiote_out_task_enable(&gpiote, gpio_nrf);

    uint32_t task_set_addr = nrfx_gpiote_set_task_address_get(&gpiote, gpio_nrf);
    uint32_t task_clr_addr = nrfx_gpiote_clr_task_address_get(&gpiote, gpio_nrf);

	/* Configuration of the GPIOTE is finished here */


	/* Configure timer */

    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer.p_reg);
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;

    err = nrfx_timer_init(&timer, &timer_config, timer_dummy_handler);

	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_timer_init error: 0x%08X", err);
		return 0;
	}

    uint32_t cc0_event_addr = nrfx_timer_event_address_get(&timer, NRF_TIMER_EVENT_COMPARE0);
    uint32_t cc1_event_addr = nrfx_timer_event_address_get(&timer, NRF_TIMER_EVENT_COMPARE1);


	nrfx_timer_compare(&timer, NRF_TIMER_CC_CHANNEL0, 1, false);

    uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer, 10);
    nrfx_timer_compare(&timer, NRF_TIMER_CC_CHANNEL1, desired_ticks, false);
	
	desired_ticks = nrfx_timer_us_to_ticks(&timer, 10000);
    nrfx_timer_extended_compare(&timer,
                                NRF_TIMER_CC_CHANNEL2,
                                desired_ticks,
                                NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                                false);
    nrfx_timer_clear(&timer);
    nrfx_timer_enable(&timer);

	/* Configuration of the TIMER is finished here */


	/* Configure DPPI connection */

	uint8_t ppi_ch1_zero;
    uint8_t ppi_ch2_one;

	err = nrfx_gppi_channel_alloc(&ppi_ch1_zero);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gppi_channel_alloc error: 0x%08X", err);
		return 0;
	}
	err = nrfx_gppi_channel_alloc(&ppi_ch2_one);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gppi_channel_alloc error: 0x%08X", err);
		return 0;
	}

	/* Configure connections between timer and tasks*/
    nrfx_gppi_channel_endpoints_setup(ppi_ch1_zero, cc0_event_addr, task_set_addr);
    nrfx_gppi_channel_endpoints_setup(ppi_ch2_one, cc1_event_addr, task_clr_addr);

	/* Enable the channels. */
	nrfx_gppi_channels_enable(BIT(ppi_ch1_zero));
	nrfx_gppi_channels_enable(BIT(ppi_ch2_one));

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int led_nrf_pm_action(const struct device *dev,
			     enum pm_device_action action)
{
	uint32_t gpio_nrf = *((uint32_t *)dev->config);

	if (action == PM_DEVICE_ACTION_SUSPEND)
	{
		nrfx_timer_disable(&timer);
		nrfx_gpiote_out_task_disable(&gpiote, gpio_nrf);
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
