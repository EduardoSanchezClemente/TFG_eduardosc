/*
 * Copyright (c) 2017, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the B105 Electronic Systems Lab.
 * 4. Neither the name of the B105 Electronic Systems Lab nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY UNIVERSITY AND CONTRIBUTORS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * lis3dsh_driver.h
 *
 *  Created on: 16 de ene. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file lis3dsh_driver.h
 */

#ifndef APPLICATION_USER_BSP_YETIMOTE_ACELEROMETER_LIS3DH_DRIVER_LIS3DSH_DRIVER_H_
#define APPLICATION_USER_BSP_YETIMOTE_ACELEROMETER_LIS3DH_DRIVER_LIS3DSH_DRIVER_H_

#include "accelerometer_driver.h"
#include "lis3dsh_hal.h"
#include "yetimote-conf.h"
#include "arm_math.h"
#include "os_i2c.h"

#define ACEL_2G_RESOLUTION				0.000061f
#define ACEL_4G_RESOLUTION				0.000122f
#define ACEL_8G_RESOLUTION				0.000244f
#define ACEL_16G_RESOLUTION				0.000488f


#define FIFO_THREESHOLD					20
#define MAX_ODR_AVAILABLE				9
#define LOW_POWER_MODE_AVAILABLE_ODR	9


accelerometer_driver_funcs_t lis3dsh_driver_func;

#endif /* APPLICATION_USER_BSP_YETIMOTE_ACELEROMETER_LIS3DH_DRIVER_LIS3DSH_DRIVER_H_ */
