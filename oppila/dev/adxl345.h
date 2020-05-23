/*
 * Copyright (c) 2014, OpenMote Technologies, S.L.
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
 * \addtogroup oppila omote-sensors
 * @{
 *
 * \defgroup oppila omote-adxl345-sensor ADXL345 acceleration sensor
 * @{
 *
 * \file
 * ADXL345 acceleration sensor driver header file
 *
 * \author
 * Pere Tuset <peretuset@openmote.com>
 */
/*---------------------------------------------------------------------------*/
#ifndef ADXL345_H_
#define ADXL345_H_
/*---------------------------------------------------------------------------*/
#define ADXL345_ERROR             (-1)
#define ADXL345_SUCCESS           (0)
#define ADXL345_ACTIVATE          (SENSORS_ACTIVE)
#define ADXL345_READ_X            (2)
#define ADXL345_READ_X_mG         (3)
#define ADXL345_READ_Y            (4)
#define ADXL345_READ_Y_mG         (5)
#define ADXL345_READ_Z            (6)
#define ADXL345_READ_Z_mG         (7)
#define ADXL345_CALIB_OFFSET      (8)
#define ADXL345_NONE              (9)
/*---------------------------------------------------------------------------*/
#define ADXL345_SENSOR "ADXL345 Sensor"
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor adxl345;
/*---------------------------------------------------------------------------*/
#endif /* ADXL345_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
