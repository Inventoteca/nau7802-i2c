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

#include "mgos_nau7802.h"

#define MGOS_NAU7802_I2CADDR_DEFAULT (0x2A)
#define MGOS_NAU7802_PU_CTRL (0x00)
#define MGOS_NAU7802_CTRL1 (0x01)
#define MGOS_NAU7802_CTRL2 (0x02)
#define MGOS_NAU7802_ADCO_B2 (0x12)
#define MGOS_NAU7802_ADC (0x15)
#define MGOS_NAU7802_PGA (0x1B)
#define MGOS_NAU7802_POWER (0x1C)
#define MGOS_NAU7802_REVISION_ID (0x1F)

#define MGOS_NAU7802_PU_CTRL_RR (0x01)
#define MGOS_NAU7802_PU_CTRL_PUD (0x02)
#define MGOS_NAU7802_PU_CTRL_PUA (0x04)
#define MGOS_NAU7802_PU_CTRL_PUR (0x08)
#define MGOS_NAU7802_PU_CTRL_CS (0x10)
#define MGOS_NAU7802_PU_CTRL_CR (0x20)
#define MGOS_NAU7802_PU_CTRL_OSCS (0x40)
#define MGOS_NAU7802_PU_CTRL_AVDDS (0x80)

#define MGOS_NAU7802_CTRL1_GAIN_OFFSET (0)
#define MGOS_NAU7802_CTRL1_GAIN_MASK (0x07)
#define MGOS_NAU7802_CTRL1_VLDO_OFFSET (3)
#define MGOS_NAU7802_CTRL1_VLDO_MASK (0x07 << MGOS_NAU7802_CTRL1_VLDO_OFFSET)
#define MGOS_NAU7802_CTRL1_DRDY_SEL (0x40)
#define MGOS_NAU7802_CTRL1_CRP (0x80)

#define MGOS_NAU7802_CTRL2_CALMOD_OFFSET_CAL_INTERNAL (0x00)
#define MGOS_NAU7802_CTRL2_CALMOD_OFFSET_CAL_SYSTEM (0x02)
#define MGOS_NAU7802_CTRL2_CALMOD_GAIN_CAL_SYSTEM (0x03)
#define MGOS_NAU7802_CTRL2_CALMOD_MASK (0x03)
#define MGOS_NAU7802_CTRL2_CALS (0x04)
#define MGOS_NAU7802_CTRL2_CAL_ERROR (0x08)
#define MGOS_NAU7802_CTRL2_CRS_OFFSET (4)
#define MGOS_NAU7802_CTRL2_CRS_MASK (0x07 << MGOS_NAU7802_CTRL2_CRS_OFFSET)
#define MGOS_NAU7802_CTRL2_CHS (0x80)

#define MGOS_NAU7802_POWER_PGA_CAP_EN (0x80)


#ifdef __cplusplus
extern "C" {
#endif

struct mgos_nau7802 {
  struct mgos_i2c *i2c;
  uint8_t i2caddr;

  int drdy_gpio;
  mgos_nau7802_irq_handler_f user_cb;
};

#ifdef __cplusplus
}
#endif