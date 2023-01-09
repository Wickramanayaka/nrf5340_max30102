/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <drivers/i2c.h>
#include <device.h>
#include <math.h>
#include "max30102.h"

void main(void)
{
	printk("The I2C scanner started\n");
    const struct device *i2c_dev;
    int error;

    i2c_dev = device_get_binding("I2C_1");
    if (!i2c_dev) {
        printk("Binding failed.");
        return;
    }

    /* Demonstration of runtime configuration */
    i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	struct i2c_msg msgs[1];
	uint8_t dst = 1;
	uint8_t i = 0x57;
	msgs[0].buf = &dst;
	msgs[0].len = 1U;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	error = i2c_transfer(i2c_dev, &msgs[0], 1, i);
	if (error == 0) {
		printk("0x%2x FOUND\n", i);
		MAX30102_check(i2c_dev);     
		MAX30102_reset(i2c_dev);    
		MAX30102_clear(i2c_dev);   
		MAX30102_config(i2c_dev);
	}
	else {
		printk("error 0x%2x \n", i);
	}

	int32_t n_spo2;
	int32_t n_heart_rate;
	int32_t aun_ir_buffer;
	int32_t aun_red_buffer;
	int8_t *ch_spo2_valid;
	int8_t *ch_hr_valid;
	int32_t n_ir_buffer_length = 32 << 0;
		
	while (1) {
		MAX30102_read_fifo(i2c_dev, &aun_red_buffer, &aun_ir_buffer);
		maxim_heart_rate_and_oxygen_saturation(&aun_ir_buffer, n_ir_buffer_length, &aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
		printk("SPO2: %d", aun_ir_buffer);
		printk(" HR: %d", aun_red_buffer);
		printk("\n");
		//k_sleep(K_MSEC(100));
	}
}
