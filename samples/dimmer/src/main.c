/*
 * Copyright (c) 2024 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>



int main(void)
{
	uint8_t dim_value = 0;
	const struct device *led_nrf;
	led_nrf = DEVICE_DT_GET(DT_NODELABEL(nrf_led));
	while (dim_value < 100)
	{
		led_set_brightness(led_nrf, 0, dim_value++);
		k_sleep(K_MSEC(50));
	}


	pm_device_action_run(led_nrf, PM_DEVICE_ACTION_SUSPEND);

	return 0;
}
