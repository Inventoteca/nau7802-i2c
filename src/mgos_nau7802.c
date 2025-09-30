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

#include "mgos_nau7802_internal.h"

static void mgos_nau7802_irq_handler(int pin, void *arg) {
  struct mgos_nau7802 *dev = (struct mgos_nau7802 *) arg;

  if (dev->user_cb) {
    dev->user_cb(dev);
  }
}

bool mgos_nau7802_check_revision_id(struct mgos_nau7802 *dev) {
  if (!dev) {
    return 0;
  }

  int revision_id = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_REVISION_ID);
  if (revision_id != 0xf) {
    LOG(LL_ERROR, ("Error reading revision ID register: expected 0xf, got=%#02x", revision_id));
  }

  return (revision_id == 0xf);
}

bool mgos_nau7802_init(struct mgos_nau7802 *dev) {
  if (!dev) goto init_failed;

  if (!mgos_nau7802_reset(dev)) goto init_failed;
  if (!mgos_nau7802_power_up(dev)) goto init_failed;
  if (!mgos_nau7802_set_ldo(dev, MGOS_NAU7802_LDO_3V3)) goto init_failed;
  if (!mgos_nau7802_set_gain(dev, MGOS_NAU7802_GAIN_16)) goto init_failed;
  if (!mgos_nau7802_set_sps(dev, MGOS_NAU7802_SPS_10)) goto init_failed;

  //default values recomended on datasheet
  //MGOS_NAU7802_ADC <- REG_CHPS[5:4]=11
  mgos_i2c_write_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_ADC, 0x30);
  //enable decoupling cap on chan 2
  uint8_t pwr = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_POWER);
  mgos_i2c_write_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_POWER,
                       pwr | MGOS_NAU7802_POWER_PGA_CAP_EN);

  if (!mgos_nau7802_calibrate(dev)) goto init_failed;
  if (!mgos_nau7802_check_revision_id(dev)) goto init_failed;

  return true;
init_failed:
  LOG(LL_ERROR, ("Error initializing NAU7802"));
  return false;
}

struct mgos_nau7802 *mgos_nau7802_create(struct mgos_i2c *i2c) {
  struct mgos_nau7802 *dev = NULL;

  if (!i2c) goto create_failed;

  dev = calloc(1, sizeof(struct mgos_nau7802));
  if (!dev) goto create_failed;

  memset(dev, 0, sizeof(struct mgos_nau7802));
  dev->i2caddr = MGOS_NAU7802_I2CADDR_DEFAULT;
  dev->i2c = i2c;
  dev->drdy_gpio = -1;
  dev->user_cb = 0;

  if (!mgos_nau7802_init(dev)) goto create_failed;

  LOG(LL_INFO, ("NAU7802 initialized at I2C %#02x", dev->i2caddr));
  return dev;
create_failed:
  free(dev);
  return NULL;
}

bool mgos_nau7802_destroy(struct mgos_nau7802 **dev) {
  if (!*dev) {
    return false;
  }

  mgos_nau7802_disable_interrupt(*dev);

  free(*dev);
  *dev = NULL;
  return true;
}

bool mgos_nau7802_enable_interrupt(struct mgos_nau7802 *dev, int drdy_gpio, mgos_nau7802_irq_handler_f cb) {
  if (!dev) {
    return false;
  }

  dev->drdy_gpio = drdy_gpio;
  dev->user_cb = cb;

  if (dev->drdy_gpio != -1) {
    LOG(LL_INFO, ("Installing interrupts on GPIO %d", dev->drdy_gpio));
    mgos_gpio_set_mode(dev->drdy_gpio, MGOS_GPIO_MODE_INPUT);
    mgos_gpio_set_pull(dev->drdy_gpio, MGOS_GPIO_PULL_UP);
    mgos_gpio_set_int_handler(dev->drdy_gpio, MGOS_GPIO_INT_EDGE_POS,
                              mgos_nau7802_irq_handler, dev);
    mgos_gpio_clear_int(dev->drdy_gpio);
    mgos_gpio_enable_int(dev->drdy_gpio);
  }

  return true;
}

bool mgos_nau7802_disable_interrupt(struct mgos_nau7802 *dev) {
  if (!dev) {
    return false;
  }

  if (dev->drdy_gpio != -1) {
    LOG(LL_INFO, ("Disabling interrupts on GPIO %d", dev->drdy_gpio));
    mgos_gpio_disable_int(dev->drdy_gpio);
    mgos_gpio_clear_int(dev->drdy_gpio);
    mgos_gpio_remove_int_handler(dev->drdy_gpio, NULL, NULL);
  }

  dev->drdy_gpio = -1;
  dev->user_cb = 0;

  return true;
}

bool mgos_nau7802_reset(struct mgos_nau7802 *dev) {
  if (!dev) {
    return false;
  }

  mgos_i2c_write_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL, MGOS_NAU7802_PU_CTRL_RR);
  mgos_usleep(1);
  mgos_i2c_write_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL, 0);

  return true;
}

bool mgos_nau7802_power_up(struct mgos_nau7802 *dev) {
  if (!dev) {
    return false;
  }

  uint8_t pu_ctlr = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL);
  mgos_i2c_write_reg_b(
      dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL,
      pu_ctlr | MGOS_NAU7802_PU_CTRL_PUD | MGOS_NAU7802_PU_CTRL_PUA);
  //power up takes ~200us
  uint8_t i = 0;
  do {
    if (i++ > 10) {  //max wait: 100us*10=1ms
      LOG(LL_ERROR, ("Error powering up NAU7802"));
      return false;
    }

    mgos_usleep(100);
    pu_ctlr = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL);
  } while (!(pu_ctlr & MGOS_NAU7802_PU_CTRL_PUR));

  return true;
}

bool mgos_nau7802_power_down(struct mgos_nau7802 *dev) {
  if (!dev) {
    return false;
  }

  uint8_t pu_ctlr = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL);
  mgos_i2c_write_reg_b(
      dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL,
      pu_ctlr & ~(MGOS_NAU7802_PU_CTRL_PUD | MGOS_NAU7802_PU_CTRL_PUR));

  return true;
}

bool mgos_nau7802_set_ldo(struct mgos_nau7802 *dev, uint8_t ldo_value) {
  if (!dev) {
    return false;
  }

  //set LDO value
  uint8_t ctlr1 = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL1);
  ctlr1 &= ~MGOS_NAU7802_CTRL1_VLDO_MASK;
  mgos_i2c_write_reg_b(
      dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL1,
      ctlr1 | ((ldo_value << MGOS_NAU7802_CTRL1_VLDO_OFFSET) & MGOS_NAU7802_CTRL1_VLDO_MASK));

  //enable internal LDO
  uint8_t pu_ctlr = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL);
  mgos_i2c_write_reg_b(
      dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL,
      pu_ctlr | MGOS_NAU7802_PU_CTRL_AVDDS);

  return true;
}

bool mgos_nau7802_set_gain(struct mgos_nau7802 *dev, uint8_t gain_value) {
  if (!dev) {
    return false;
  }

  //set gain value
  uint8_t ctlr1 = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL1);
  ctlr1 &= ~MGOS_NAU7802_CTRL1_GAIN_MASK;
  mgos_i2c_write_reg_b(
      dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL1,
      ctlr1 | ((gain_value << MGOS_NAU7802_CTRL1_GAIN_OFFSET) & MGOS_NAU7802_CTRL1_GAIN_MASK));

  return true;
}

bool mgos_nau7802_set_sps(struct mgos_nau7802 *dev, uint8_t sps_value) {
  if (!dev) {
    return false;
  }

  //set crs value
  uint8_t ctlr2 = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL2);
  ctlr2 &= ~MGOS_NAU7802_CTRL2_CRS_MASK;
  mgos_i2c_write_reg_b(
      dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL2,
      ctlr2 | ((sps_value << MGOS_NAU7802_CTRL2_CRS_OFFSET) & MGOS_NAU7802_CTRL2_CRS_MASK));

  return true;
}

bool mgos_nau7802_calibrate(struct mgos_nau7802 *dev) {
  if (!dev) {
    return false;
  }

  //init calibration: assumed CALMOD=00=Offset Calibration Internal
  uint8_t ctlr2 = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL2);
  mgos_i2c_write_reg_b(
      dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL2,
      ctlr2 | MGOS_NAU7802_CTRL2_CALS);

  //wait for MGOS_NAU7802_CTRL2_CALS change to 0
  uint8_t i = 0;
  do {
    if (i++ > 10) {  //max wait: 100ms*10=1s. on tests it takes ~400ms
      LOG(LL_ERROR, ("Error calibrating up NAU7802: timed out"));
      return false;
    }

    mgos_msleep(100);
    ctlr2 = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_CTRL2);
  } while (ctlr2 & MGOS_NAU7802_CTRL2_CALS);

  if (ctlr2 & MGOS_NAU7802_CTRL2_CAL_ERROR) {
    LOG(LL_ERROR, ("Error calibrating NAU7802: CTRL2_CAL_ERROR"));
    return false;
  }

  return true;
}

bool mgos_nau7802_data_ready(struct mgos_nau7802 *dev) {
  if (!dev) {
    return false;
  }

  uint8_t pu_ctlr = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, MGOS_NAU7802_PU_CTRL);

  return (pu_ctlr & MGOS_NAU7802_PU_CTRL_CR);
}

bool mgos_nau7802_wait_data_ready(struct mgos_nau7802 *dev, uint32_t delay_ms, uint8_t max_attempts) {
  if (!dev) {
    return false;
  }

  uint8_t i = 0;
  while (!mgos_nau7802_data_ready(dev)) {
    if (i++ > max_attempts) {  //max wait: delay_ms*max_attempts
      LOG(LL_ERROR, ("Error waiting for data on NAU7802: timed out"));
      return false;
    }
    mgos_msleep(delay_ms);
  }

  return true;
}

int32_t mgos_nau7802_read(struct mgos_nau7802 *dev) {
  if (!dev) {
    return 0;
  }

  uint8_t buf[3];  //MSB first
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, MGOS_NAU7802_ADCO_B2, 3, buf)) {
    LOG(LL_ERROR, ("Error reading data from NAU7802"));
    return 0;
  }

  uint32_t value;
  value = (buf[0] << 16) | (buf[1] << 8) | buf[2];
  if (value & 0x800000) {  //extend sign bit
    value |= 0xFF000000;
  }

  return (int32_t) value;
}

// Mongoose OS library initialization
bool mgos_nau7802_i2c_init(void) {
  return true;
}