/*
 * Copyright (c) 2016, Zolertia
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
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup OMote
 * @{
 *
 * \defgroup OMote
 *
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other OMote peripherals
 *
 * This file can be used as the basis to configure other platforms using the
 * cc2538 SoC.
 * @{
 *
 * There are on board sensors like LDR, BMP180 and ADXL335 
 *
 * \file
 * Header file with definitions related to the I/O connections on the Oppila Microsystems's
 * OMote, cc2538-based
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
#ifndef BOARD_H_
#define BOARD_H_

#include "dev/gpio.h"
#include "dev/nvic.h"
/* ----------------------+---+---+---------------------------------------------
 */
/*---------------------------------------------------------------------------*/
/** \name OMote LED configuration
 *
 * LEDs on the OMote 
 * - LED1 (Green)    -> PD0
 * - LED2 (Green)    -> PD4 
 *
 * The LEDs are connected to a MOSFET to minimize current draw.  The LEDs can
 * be disabled by removing resistors R9 and R12
 * @{
 */
/*---------------------------------------------------------------------------*/
#define LEDS_ARCH_L1_PORT GPIO_D_NUM
#define LEDS_ARCH_L1_PIN  0
#define LEDS_ARCH_L2_PORT GPIO_D_NUM
#define LEDS_ARCH_L2_PIN  4

#define LEDS_CONF_GREEN     1
#define LEDS_CONF_GREEN1    2

#define LEDS_CONF_COUNT   2
/** @} */
/*---------------------------------------------------------------------------*/
/** \name USB configuration
 *
 * The USB pullup is enabled by an external resistor, not mapped to a GPIO
 */
#ifdef USB_PULLUP_PORT
#undef USB_PULLUP_PORT
#endif
#ifdef USB_PULLUP_PIN
#undef USB_PULLUP_PIN
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/** \name UART configuration
 *
 * On the OMote, the UARTs are connected to the following ports/pins:
 *
 * - UART0:
 *   - RX:  PA0
 *   - TX:  PA1
 * - UART1:
 *   - RX:  PC1
 *   - TX:  PC0
 *   - CTS: disabled as default, PD0 may be assigned if not using I2C interrupts
 *   - RTS: disabled as default
 *
 * We configure the port to use UART0 and UART1, CTS/RTS only for UART1,
 * both without a HW pull-up resistor
 * @{
 */
#define UART0_RX_PORT            GPIO_A_NUM
#define UART0_RX_PIN             0
#define UART0_TX_PORT            GPIO_A_NUM
#define UART0_TX_PIN             1

#define UART1_RX_PORT            GPIO_C_NUM
#define UART1_RX_PIN             1
#define UART1_TX_PORT            GPIO_C_NUM
#define UART1_TX_PIN             0
#define UART1_CTS_PORT           (-1)
#define UART1_CTS_PIN            (-1)
#define UART1_RTS_PORT           (-1)
#define UART1_RTS_PIN            (-1)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ADC configuration
 *
 * These values configure which CC2538 pins and ADC channels to use for the ADC
 * inputs. By default the OMote allows four out-of-the-box ADC ports with a
 * phidget-like 3-pin connector (GND/VDD/ADC)
 *
 * The OMote allows both 3.3V and 5V analogue sensors as follow:
 *
 *
 * - ADC2 (PA4): up to 3.3V
 * - ADC3 (PA2): up to 5V, by means of a 2/3 voltage divider.
 *
 * Also there are other ADC channels shared as gpio pins and
 * user bsl button implementations:
 *
 * - ADC4 (PA6): up to 3.3V.
 * - ADC5 (PA7): up to 3.3V.
 * - ADC6 (PA3): up to 3.3V.
 *
 * ADC inputs can only be on port A.
 *
 * The internal ADC reference is 1190mV, use either a voltage divider as input,
 * or a different voltage reference, like AVDD5, or externally using PA7/AIN7
 * and PA6/AIN6 configurable as differential reference, by removing the R26 and
 * R33 0Ohm resistors to disconnect off the Micro-SD, and those will be
 * accessible from JP5 connector.
 *
 * To enable the ADC[2,4-6], remove any 0Ohm resistors if required (see above),
 * and define in your application `ADC_SENSORS_CONF_ADCx_PIN` and set its
 * value with the corresponding pin number (i.e ADC2 to 4 as mapped to PA4).
 * To disable any ADC[1-6] just define as above, but set to (-1) instead.

 * Warning: if using ADC6 (PA3), you will need to disable the bootloader by
 * making FLASH_CCA_CONF_BOOTLDR_BACKDOOR equal to zero
 * 
 * @{
 */
#define ADC_SENSORS_PORT         GPIO_A_NUM    /**< ADC GPIO control port */

#ifndef ADC_SENSORS_CONF_ADC1_PIN
#define ADC_SENSORS_ADC1_PIN     5             /**< ADC1 to PA5, 3V3    */
#else
#if ((ADC_SENSORS_CONF_ADC1_PIN != -1) && (ADC_SENSORS_CONF_ADC1_PIN != 5))
#error "ADC1 channel should be mapped to PA5 or disabled with -1"
#else						/**< LDR sensor is connected to ADC1 */
#define ADC_SENSORS_ADC1_PIN ADC_SENSORS_CONF_ADC1_PIN   
#endif
#endif

#ifndef ADC_SENSORS_CONF_ADC3_PIN
#define ADC_SENSORS_ADC3_PIN     2             /**< ADC3 to PA2, 5V     */
#else
#if ((ADC_SENSORS_CONF_ADC3_PIN != -1) && (ADC_SENSORS_CONF_ADC3_PIN != 2))
#error "ADC3 channel should be mapped to PA2 or disabled with -1"
#else
#define ADC_SENSORS_ADC3_PIN ADC_SENSORS_CONF_ADC3_PIN
#endif
#endif

#ifndef ADC_SENSORS_CONF_ADC2_PIN
#define ADC_SENSORS_ADC2_PIN     (-1)          /**< ADC2 no declared    */
#else
#define ADC_SENSORS_ADC2_PIN     4             /**< Hard-coded to PA4    */
#endif

#ifndef ADC_SENSORS_CONF_ADC4_PIN
#define ADC_SENSORS_ADC4_PIN     (-1)          /**< ADC4 not declared    */
#else
#define ADC_SENSORS_ADC4_PIN     6             /**< Hard-coded to PA6    */
#endif

#ifndef ADC_SENSORS_CONF_ADC5_PIN
#define ADC_SENSORS_ADC5_PIN     (-1)          /**< ADC5 not declared    */
#else
#define ADC_SENSORS_ADC5_PIN     7             /**< Hard-coded to PA7    */
#endif

#ifndef ADC_SENSORS_CONF_ADC6_PIN
#define ADC_SENSORS_ADC6_PIN     (-1)          /**< ADC6 not declared    */
#else
#define ADC_SENSORS_ADC6_PIN     3             /**< Hard-coded to PA3    */
#endif

#ifndef ADC_SENSORS_CONF_MAX
#define ADC_SENSORS_MAX          2             /**< Maximum sensors    */
#else
#define ADC_SENSORS_MAX          ADC_SENSORS_CONF_MAX
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/** \name OMote Button configuration
 *
 * Buttons on the OMote are connected as follows:
 * - BUTTON_USER  -> PB0, S1 user button,
 * - BUTTON_BSL   -> PA3, bootloader button
 * - BUTTON_RESET -> RESET_N line, S2 reset the CC2538
 * 
 * @{
 */
#define BUTTON_USER_PORT       GPIO_B_NUM
#define BUTTON_USER_PIN        0
#define BUTTON_USER_VECTOR     GPIO_B_IRQn

/* Notify various examples that we have an user button.
 */
/*#ifdef PLATFORM_CONF_WITH_BUTTON
#if (PLATFORM_CONF_WITH_BUTTON && (ADC_SENSORS_ADC6_PIN == 3))
#error "The ADC6 (PA3) and user button cannot be enabled at the same time" 
#else
#define PLATFORM_HAS_BUTTON  (PLATFORM_CONF_WITH_BUTTON && \
                              !(ADC_SENSORS_ADC6_PIN == 3))
#endif */ /* (PLATFORM_CONF_WITH_BUTTON && (ADC_SENSORS_ADC6_PIN == 3)) */
/*#else
#define PLATFORM_HAS_BUTTON  !(ADC_SENSORS_ADC6_PIN == 3)
#endif */ /* PLATFORM_CONF_WITH_BUTTON */


#define PLATFORM_HAS_BUTTON     1
#define PLATFORM_SUPPORTS_BUTTON_HAL 1



/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SPI (SSI1) configuration
 *
 * These values configure which CC2538 pins to use for the SPI (SSI1) lines
 * TX -> MOSI, RX -> MISO
 * @{
 */
#define SPI1_CLK_PORT            GPIO_C_NUM
#define SPI1_CLK_PIN             4
#define SPI1_TX_PORT             GPIO_C_NUM
#define SPI1_TX_PIN              5
#define SPI1_RX_PORT             GPIO_C_NUM
#define SPI1_RX_PIN              6
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name I2C configuration
 *
 * These values configure which CC2538 pins to use for the I2C lines, exposed
 * over JP6 connector.
 * 
 * The I2C is exposed over the JP6 header, using a 5-pin connector with 2.54 mm
 * spacing, providing also D+3.3V, GND and PD0 pin that can be used as an
 * interrupt pin if required
 * @{
 */
#define I2C_SCL_PORT             GPIO_C_NUM
#define I2C_SCL_PIN              3
#define I2C_SDA_PORT             GPIO_C_NUM
#define I2C_SDA_PIN              2
#define I2C_INT_PORT             GPIO_D_NUM
#define I2C_INT_PIN              0
#define I2C_INT_VECTOR           GPIO_D_IRQn
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name CC2538 TSCH configuration
 *
 * @{
 */
#define RADIO_PHY_OVERHEAD        CC2538_PHY_OVERHEAD
#define RADIO_BYTE_AIR_TIME       CC2538_BYTE_AIR_TIME
#define RADIO_DELAY_BEFORE_TX     CC2538_DELAY_BEFORE_TX
#define RADIO_DELAY_BEFORE_RX     CC2538_DELAY_BEFORE_RX
#define RADIO_DELAY_BEFORE_DETECT CC2538_DELAY_BEFORE_DETECT
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "Oppila Microsystems OMote platform"
/** @} */

#endif /* BOARD_H_ */

/**
 * @}
 * @}
 */
