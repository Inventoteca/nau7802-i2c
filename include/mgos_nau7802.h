/*
 * Copyright 2021 Turbine Kreuzberg
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "mgos.h"
#include "mgos_i2c.h"

#define MGOS_NAU7802_LDO_4V5 (0x00)
#define MGOS_NAU7802_LDO_4V2 (0x01)
#define MGOS_NAU7802_LDO_3V9 (0x02)
#define MGOS_NAU7802_LDO_3V6 (0x03)
#define MGOS_NAU7802_LDO_3V3 (0x04)
#define MGOS_NAU7802_LDO_3V0 (0x05)
#define MGOS_NAU7802_LDO_2V7 (0x06)
#define MGOS_NAU7802_LDO_2V4 (0x07)

#define MGOS_NAU7802_GAIN_0 (0x00)
#define MGOS_NAU7802_GAIN_2 (0x01)
#define MGOS_NAU7802_GAIN_4 (0x02)
#define MGOS_NAU7802_GAIN_8 (0x03)
#define MGOS_NAU7802_GAIN_16 (0x04)
#define MGOS_NAU7802_GAIN_32 (0x05)
#define MGOS_NAU7802_GAIN_64 (0x06)
#define MGOS_NAU7802_GAIN_128 (0x07)

#define MGOS_NAU7802_SPS_10 (0x00)
#define MGOS_NAU7802_SPS_20 (0x01)
#define MGOS_NAU7802_SPS_40 (0x02)
#define MGOS_NAU7802_SPS_80 (0x03)
#define MGOS_NAU7802_SPS_320 (0x07)

#ifdef __cplusplus
extern "C" {
#endif

struct mgos_nau7802;

typedef void (*mgos_nau7802_irq_handler_f)(struct mgos_nau7802 *dev);

/**
 * Initialize a NAU7802 on the I2C bus `i2c` at address 0x2A (fixed on hardware),
 * upon success a new `struct mgos_nau7802` is allocated and returned. 
 * If the device could not be initialized, NULL is returned.
 * 
 * Device is initialized with recomended values from datasheet 
 * and MGOS_NAU7802_LDO_3V3, MGOS_NAU7802_GAIN_128 and MGOS_NAU7802_SPS_10
 */
struct mgos_nau7802 *mgos_nau7802_create(struct mgos_i2c *i2c);

/**
 * Destroy the data structure associated with a NAU7802 device. The reference
 * to the pointer of the `struct mgos_nau7802` has to be provided, and upon
 * successful destruction, its associated memory will be freed and the pointer
 * set to NULL.
 */
bool mgos_nau7802_destroy(struct mgos_nau7802 **dev);

/**
 * Enable interrupt handling on pin `drdy_gpio`. On negative edge user cb will be triggered.
 * User should clear interrupt by reading the device.
 */
bool mgos_nau7802_enable_interrupt(struct mgos_nau7802 *dev, int drdy_gpio, mgos_nau7802_irq_handler_f cb);

/*
 * Disable interrupt handling
 */
bool mgos_nau7802_disable_interrupt(struct mgos_nau7802 *dev);

/**
 * Perform a device reset
 */
bool mgos_nau7802_reset(struct mgos_nau7802 *dev);

/**
 * Power up device
 */
bool mgos_nau7802_power_up(struct mgos_nau7802 *dev);

/**
 * Power down device
 */
bool mgos_nau7802_power_down(struct mgos_nau7802 *dev);

/**
 * Set analog voltage regulator level
 */
bool mgos_nau7802_set_ldo(struct mgos_nau7802 *dev, uint8_t ldo_value);

/**
 * Set amplifier gain
 */
bool mgos_nau7802_set_gain(struct mgos_nau7802 *dev, uint8_t gain_value);

/**
 * Set ADC sampling frequency
 */
bool mgos_nau7802_set_sps(struct mgos_nau7802 *dev, uint8_t sps_value);

/**
 * Perform internal calibration routine
 */
bool mgos_nau7802_calibrate(struct mgos_nau7802 *dev);

/**
 * Check data ready
 */
bool mgos_nau7802_data_ready(struct mgos_nau7802 *dev);

/**
 * Wait for data ready. Function will block a maximum of `delay_ms * max_attepmts`
 */ 
bool mgos_nau7802_wait_data_ready(struct mgos_nau7802 *dev, uint32_t delay_ms, uint8_t max_attempts);

/**
 * Get one ADC reading. User should check for data ready
 */
int32_t mgos_nau7802_read(struct mgos_nau7802 *dev);

#ifdef __cplusplus
}
#endif