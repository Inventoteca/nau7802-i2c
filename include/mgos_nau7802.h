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

struct mgos_nau7802 *mgos_nau7802_create(struct mgos_i2c *i2c, int reset_gpio);
bool mgos_nau7802_destroy(struct mgos_nau7802 **dev);

bool mgos_nau7802_init(struct mgos_nau7802 *dev);

bool mgos_nau7802_reset(struct mgos_nau7802 *dev);
bool mgos_nau7802_power_up(struct mgos_nau7802 *dev);
bool mgos_nau7802_power_down(struct mgos_nau7802 *dev);
bool mgos_nau7802_set_ldo(struct mgos_nau7802 *dev, uint8_t ldo_value);
bool mgos_nau7802_set_gain(struct mgos_nau7802 *dev, uint8_t gain_value);
bool mgos_nau7802_set_sps(struct mgos_nau7802 *dev, uint8_t sps_value);


#ifdef __cplusplus
}
#endif