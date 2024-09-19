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

	return 0;
}
