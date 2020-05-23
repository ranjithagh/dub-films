/*
 * Copyright (c) 2020, OPPILA MICROSYSTEMS, Banglore
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup Oppila Omote-adxl345-sensor
 * @{
 *
 * \file
 * Driver for the ADXL345 acceleration sensor
 *
 * 
 * 
 */
/*---------------------------------------------------------------------------*/
#include "dev/i2c.h"
#include "dev/adxl345.h"
#include "lib/sensors.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/**
 * \name ADXL345 address and device identifier
 * @{
 */
#define ADXL345_ADDRESS                     (0x53)
#define ADXL345_DEVID_VALUE                 (0xE6)
/** @} */
/* -------------------------------------------------------------------------- */
/**
 * \name ADXL345 register addresses
 * @{
 */
#define ADXL345_DEVID_ADDR                  (0x00)
#define ADXL345_THRES_TAP_ADDR              (0x1D)
#define ADXL345_OFSX_ADDR                   (0x1E)
#define ADXL345_OFSY_ADDR                   (0x1F)
#define ADXL345_OFSZ_ADDR                   (0x20)
#define ADXL345_DUR_ADDR                    (0x21)
#define ADXL345_LATENT_ADDR                 (0x22)
#define ADXL345_WINDOW_ADDR                 (0x23)
#define ADXL345_THRESH_ACT_ADDR             (0x24)
#define ADXL345_THRESH_INACT_ADDR           (0x25)
#define ADXL345_TIME_INACT_ADDR             (0x26)
#define ADXL345_ACT_INACT_CTL_ADDR          (0x27)
#define ADXL345_THRESH_FF_ADDR              (0x28)
#define ADXL345_TIME_FF_ADDR                (0x29)
#define ADXL345_TAP_AXES_ADDR               (0x2A)
#define ADXL345_ACT_TAP_STATUS_ADDR         (0x2B)
#define ADXL345_BW_RATE_ADDR                (0x2C)
#define ADXL345_POWER_CTL_ADDR              (0x2D)
#define ADXL345_INT_ENABLE_ADDR             (0x2E)
#define ADXL345_INT_MAP_ADDR                (0x2F)
#define ADXL345_INT_SOURCE_ADDR             (0x30)
#define ADXL345_DATA_FORMAT_ADDR            (0x31)
#define ADXL345_DATAX0_ADDR                 (0x32)
#define ADXL345_DATAX1_ADDR                 (0x33)
#define ADXL345_DATAY0_ADDR                 (0x34)
#define ADXL345_DATAY1_ADDR                 (0x35)
#define ADXL345_DATAZ0_ADDR                 (0x36)
#define ADXL345_DATAZ1_ADDR                 (0x37)
#define ADXL345_FIFO_CTL_ADDR               (0x38)
#define ADXL345_FIFO_STATUS_ADDR            (0x39)
#define ADXL345_TAP_SIGN_ADDR               (0x3A)
#define ADXL345_ORIENT_CONF_ADDR            (0x3B)
#define ADXL345_ORIENT_ADDR                 (0x3C)
/** @} */
/* -------------------------------------------------------------------------- */
/**
 * \name ADXL345 register values
 * @{
 */
#define ADXL345_INT_ENABLE_DATA_READY      (1 << 7)
#define ADXL345_INT_ENABLE_SINGLE_TAP      (1 << 6)
#define ADXL345_INT_ENABLE_DOUBLE_TAP      (1 << 5)
#define ADXL345_INT_ENABLE_ACTIVITY        (1 << 4)
#define ADXL345_INT_ENABLE_INACTIVITY      (1 << 3)
#define ADXL345_INT_ENABLE_FREE_FALL       (1 << 2)
#define ADXL345_INT_ENABLE_WATERMARK       (1 << 1)
#define ADXL345_INT_ENABLE_OVERRUN         (1 << 0)

#define ADXL345_ACT_INACT_CTL_ACT_ACDC     (1 << 7)
#define ADXL345_ACT_INACT_CTL_ACT_X_EN     (1 << 6)
#define ADXL345_ACT_INACT_CTL_ACT_Y_EN     (1 << 5)
#define ADXL345_ACT_INACT_CTL_ACT_Z_EN     (1 << 4)
#define ADXL345_ACT_INACT_CTL_INACT_ACDC   (1 << 3)
#define ADXL345_ACT_INACT_CTL_INACT_X_EN   (1 << 2)
#define ADXL345_ACT_INACT_CTL_INACT_Y_EN   (1 << 1)
#define ADXL345_ACT_INACT_CTL_INACT_Z_EN   (1 << 0)

#define ADXL345_TAP_AXES_SUPPRESS           (1 << 3)
#define ADXL345_TAP_AXES_TAP_X_EN           (1 << 2)
#define ADXL345_TAP_AXES_TAP_Y_EN           (1 << 1)
#define ADXL345_TAP_AXES_TAP_Z_EN           (1 << 0)

#define ADXL345_ACT_TAP_STATUS_ACT_X_SRC    (1 << 6)
#define ADXL345_ACT_TAP_STATUS_ACT_Y_SRC    (1 << 5)
#define ADXL345_ACT_TAP_STATUS_ACT_Z_SRC    (1 << 4)
#define ADXL345_ACT_TAP_STATUS_ASLEEP       (1 << 3)
#define ADXL345_ACT_TAP_STATUS_TAP_X_SRC    (1 << 2)
#define ADXL345_ACT_TAP_STATUS_TAP_Y_SRC    (1 << 1)
#define ADXL345_ACT_TAP_STATUS_TAP_Z_SRC    (1 << 0)

#define ADXL345_BW_RATE_POWER               (1 << 4)
#define ADXL345_BW_RATE_RATE(x)             ((x) & 0x0F)

#define ADXL345_POWER_CTL_LINK              (1 << 5)
#define ADXL345_POWER_CTL_AUTO_SLEEP        (1 << 4)
#define ADXL345_POWER_CTL_MEASURE           (1 << 3)
#define ADXL345_POWER_CTL_SLEEP             (1 << 2)
#define ADXL345_POWER_CTL_WAKEUP(x)         ((x) & 0x03)

#define ADXL345_DATA_FORMAT_SELF_TEST       (1 << 7)
#define ADXL345_DATA_FORMAT_SPI             (1 << 6)
#define ADXL345_DATA_FORMAT_INT_INVERT      (1 << 5)
#define ADXL345_DATA_FORMAT_FULL_RES        (1 << 3)
#define ADXL345_DATA_FORMAT_JUSTIFY         (1 << 2)
#define ADXL345_DATA_FORMAT_RANGE(x)        ((x) & 0x03)
#define ADXL345_DATA_FORMAT_RANGE_PM_2g     (0)
#define ADXL345_DATA_FORMAT_RANGE_PM_4g     (1)
#define ADXL345_DATA_FORMAT_RANGE_PM_8g     (2)
#define ADXL345_DATA_FORMAT_RANGE_PM_16g    (3)

#define ADXL345_USER_CONFIGURATION          (ADXL345_DATA_FORMAT_RANGE_PM_2g)

/** @} */
/*---------------------------------------------------------------------------*/
static uint8_t enabled;
/*---------------------------------------------------------------------------*/
static void
adxl345_init(void)
{
  uint8_t config[2];

  config[0] = ADXL345_BW_RATE_ADDR;
  config[1] = (ADXL345_BW_RATE_RATE(6));
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));

  config[0] = ADXL345_DATA_FORMAT_ADDR;
  config[1] = (ADXL345_USER_CONFIGURATION);
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));

  config[0] = ADXL345_POWER_CTL_ADDR;
  config[1] = (ADXL345_POWER_CTL_MEASURE);
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));
}
/*---------------------------------------------------------------------------*/
static uint8_t
adxl345_is_present(void)
{
  uint8_t is_present;

  i2c_single_send(ADXL345_ADDRESS, ADXL345_DEVID_ADDR);
  i2c_single_receive(ADXL345_ADDRESS, &is_present);

  return is_present == ADXL345_DEVID_VALUE;
}
/*---------------------------------------------------------------------------*/
static int16_t
adxl345_read_accel(uint8_t addr1, uint8_t addr2)
{
  uint8_t acceleration[2];
  int16_t result;

  i2c_single_send(ADXL345_ADDRESS, addr1);
  i2c_single_receive(ADXL345_ADDRESS, &acceleration[0]);
  i2c_single_send(ADXL345_ADDRESS, addr2);
  i2c_single_receive(ADXL345_ADDRESS, &acceleration[1]);

  result = (acceleration[1] << 8) | acceleration[0];

  return result;
}
/*---------------------------------------------------------------------------*/
static int16_t
adxl345_convert_accel(int16_t accel)
{
  int32_t result;

  result = (1000 * accel) / 256;

  return (int16_t)result;
}
/*---------------------------------------------------------------------------*/
static void
adxl345_calibrate_offset(void)
{
  int32_t accum_x = 0;
  int32_t accum_y = 0;
  int32_t accum_z = 0;
  uint8_t config[2];
  int8_t offset;

  config[0] = ADXL345_OFSX_ADDR;
  config[1] = 0;
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));
  config[0] = ADXL345_OFSY_ADDR;
  config[1] = 0;
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));
  config[0] = ADXL345_OFSZ_ADDR;
  config[1] = 0;
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));

  uint16_t i;
  for(i = 0; i < 100; i++) {
    uint16_t x, y, z;

    x = adxl345_read_accel(ADXL345_DATAX0_ADDR, ADXL345_DATAX1_ADDR);
    accum_x += x;

    y = adxl345_read_accel(ADXL345_DATAY0_ADDR, ADXL345_DATAY1_ADDR);
    accum_y += y;

    z = adxl345_read_accel(ADXL345_DATAZ0_ADDR, ADXL345_DATAZ1_ADDR);
    accum_z += z;
  }

  offset = (64 * accum_x) / 25600;
  config[0] = ADXL345_OFSX_ADDR;
  config[1] = -offset;
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));
  PRINTF("ADXL345: X calibration offset is %d\n", offset);

  offset = (64 * accum_y) / 25600;
  config[0] = ADXL345_OFSY_ADDR;
  config[1] = -offset;
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));
  PRINTF("ADXL345: Y calibration offset is %d\n", offset);

  offset = (64 * accum_z) / 25600;
  config[0] = ADXL345_OFSZ_ADDR;
  config[1] = -offset;
  i2c_burst_send(ADXL345_ADDRESS, config, sizeof(config));
  PRINTF("ADXL345: Z calibration offset is %d\n", offset);
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return enabled;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  int16_t accel;
  if(!enabled) {
    PRINTF("ADXL345: sensor not started\n");
    return ADXL345_ERROR;
  }

  if(type == ADXL345_READ_X) {
    return adxl345_read_accel(ADXL345_DATAX0_ADDR, ADXL345_DATAX1_ADDR);
  } else if(type == ADXL345_READ_Y) {
    return adxl345_read_accel(ADXL345_DATAY0_ADDR, ADXL345_DATAY1_ADDR);
  } else if(type == ADXL345_READ_Z) {
    return adxl345_read_accel(ADXL345_DATAZ0_ADDR, ADXL345_DATAZ1_ADDR);
  } else if(type == ADXL345_READ_X_mG) {
    accel = adxl345_read_accel(ADXL345_DATAX0_ADDR, ADXL345_DATAX1_ADDR);
    return adxl345_convert_accel(accel);
  } else if(type == ADXL345_READ_Y_mG) {
    accel = adxl345_read_accel(ADXL345_DATAY0_ADDR, ADXL345_DATAY1_ADDR);
    return adxl345_convert_accel(accel);
  } else if(type == ADXL345_READ_Z_mG) {
    accel = adxl345_read_accel(ADXL345_DATAZ0_ADDR, ADXL345_DATAZ1_ADDR);
    return adxl345_convert_accel(accel);
  } else {
    PRINTF("ADXL345: invalid value requested\n");
    return ADXL345_ERROR;
  }

  return ADXL345_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
  if(type == ADXL345_ACTIVATE) {
    if(!adxl345_is_present()) {
      PRINTF("ADXL345: is not present\n");
      enabled = 0;
      return ADXL345_ERROR;
    } else {
      adxl345_init();
      enabled = 1;
      return ADXL345_SUCCESS;
    }
  }

  if(type == ADXL345_CALIB_OFFSET && enabled) {
    adxl345_calibrate_offset();
    return ADXL345_SUCCESS;
  }

  return ADXL345_ERROR;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(adxl345, ADXL345_SENSOR, value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
