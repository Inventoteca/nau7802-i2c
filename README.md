# NAU7802 I2C Driver

Mongoose OS library for NAU7802 precision 24-bit ADC


## Implementation details

The Nuvoton NAU7802 is a precision low-power 24-bit analog-to-digital converter (ADC), with an onboard low-noise
programmable gain amplifier (PGA), onboard RC or Crystal oscillator, and a precision 24-bit sigma-delta (Σ-Δ)
analog to digital converter (ADC). The NAU7802 device is capable of up to 23-bit ENOB (Effective Number of Bits)
performance. This device provides a complete front-end solution for bridge/sensor measurement such as in weigh
scales, strain gauges, and many other high resolution, low sample rate applications.

### Limitations

Current implementation uses only a subset of the available capabilities: only one channel handling, no i2c streaming mode, internal calibration and internal RC clock.

## Example application
An example program using interrupts to read data from the sensor at 10Hz:
```
#include "mgos.h"
#include "mgos_nau7802.h"

#define DRDY_PIN 14

static void irq_handler(struct mgos_nau7802 *dev) {
  LOG(LL_INFO, ("reading=%d", mgos_nau7802_read(dev)));
}


enum mgos_app_init_result mgos_app_init(void) {
  struct mgos_nau7802 *dev = mgos_nau7802_create(mgos_i2c_get_global());
  if (dev == NULL) {
    LOG(LL_ERROR, ("Could not create NAU7802 device"));
    
    return MGOS_APP_INIT_ERROR;
  }

  mgos_nau7802_enable_interrupt(dev, DRDY_PIN, irq_handler);

  return MGOS_APP_INIT_SUCCESS;
}
```