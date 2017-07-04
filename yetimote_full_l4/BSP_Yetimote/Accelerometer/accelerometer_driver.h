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
 * accelerometer_driver.h
 *
 *  Created on: 16 de ene. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file accelerometer_driver.h
 */

#ifndef APPLICATION_USER_BSP_YETIMOTE_ACCELEROMETER_ACCELEROMETER_DRIVER_H_
#define APPLICATION_USER_BSP_YETIMOTE_ACCELEROMETER_ACCELEROMETER_DRIVER_H_

#include "yetimote-conf.h"
#include "arm_math.h"

#define DEFAULT_SAMPLE_NUM	64
#define MAX_SAMPLE_NUMBER	80

#define ACEL_AXIS_X	0x01
#define ACEL_AXIS_Y	0x02
#define ACEL_AXIS_Z	0x04

typedef enum accelerometer_hp_filter_state_{
	HPF_STATE_ENABLED,
	HPF_STATE_DISABLED,
}accelerometer_hp_filter_state_t;

typedef enum accelerometer_type_{
	LIS3DH,
	LIS3DSH,
	MMA8652,
	NO_ACEL_TYPE,
}accelerometer_type_t;

typedef enum accelerometer_hw_state_{
	ACEL_STATE_NORMAL,
	ACEL_STATE_LOW_POWER,
	ACEL_STATE_POWER_DOWN,
}accelerometer_hw_state_t;

typedef enum hp_cutoff_{
	HP_CUTOFF_LOWEST,
	HP_CUTOFF_LOW,
	HP_CUTOFF_HIGH,
	HP_CUTOFF_HIGHEST
}hp_cutoff_t;

typedef struct accelerometer_driver_func_ accelerometer_driver_funcs_t;

typedef struct accelerometer_driver_{
	accelerometer_type_t accel_type;
	accelerometer_hw_state_t accel_hw_state;
	uint16_t  num_available_odr;
	uint16_t* available_odr;
	uint16_t current_odr;
	uint16_t full_scale;
	float32_t resolution;
	accelerometer_hp_filter_state_t accel_hp_filter_state;
	float32_t hp_cutoff_freq;
	uint16_t hp_cutoff_factor;
	uint8_t enabled_axis;
//	int16_t last_read_values[3];
	uint16_t sample_num;
	float32_t* xAxis;
	float32_t* yAxis;
	float32_t* zAxis;

	osMutexId accelerometer_mutex_id;

	struct accelerometer_driver_func_{
		retval_t (* accel_driver_init)(struct accelerometer_driver_*);
		retval_t (* accel_driver_close)(struct accelerometer_driver_*);
		//SET FUNCS. Specific to each accelerometer driver implementation
		retval_t (* accel_driver_normal_run_mode)(struct accelerometer_driver_*);
		retval_t (* accel_driver_low_power_mode)(struct accelerometer_driver_*);
		retval_t (* accel_driver_power_down_mode)(struct accelerometer_driver_*);
		retval_t (* accel_driver_set_odr)(struct accelerometer_driver_*, uint16_t);
		retval_t (* accel_driver_set_scale)(struct accelerometer_driver_*, uint16_t);
		retval_t (* accel_driver_enable_axis)(struct accelerometer_driver_*, uint8_t);
		retval_t (* accel_driver_disable_axis)(struct accelerometer_driver_*, uint8_t);
		//READ FUNCS
		retval_t (* accel_driver_read_single_sample)(struct accelerometer_driver_*);
#if	ACEL_USE_INT_PIN
		retval_t (* accel_driver_read_stream_samples_it)(struct accelerometer_driver_*);
		retval_t (* accel_driver_interrupt_callback)(void);
#endif
		retval_t (* accel_driver_read_stream_samples_poll)(struct accelerometer_driver_*);
	}accel_funcs;
}accelerometer_driver_t;

//Driver allocation and de-allocation
accelerometer_driver_t* new_accelerometer_driver(accelerometer_type_t accel_type);
retval_t delete_accelerometer_driver(accelerometer_driver_t* accel_driver);

//These functions are common to all drivers implementations
retval_t accel_set_sample_number(accelerometer_driver_t* accel_driver, uint16_t sample_number);
uint16_t accel_get_sample_number(accelerometer_driver_t* accel_driver);
accelerometer_type_t accel_get_type(accelerometer_driver_t* accel_driver);
accelerometer_hw_state_t accel_get_run_mode(accelerometer_driver_t* accel_driver);
uint16_t accel_get_odr(accelerometer_driver_t* accel_driver);
uint16_t accel_get_scale(accelerometer_driver_t* accel_driver);
uint8_t accel_get_enabled_axis(accelerometer_driver_t* accel_driver);
float32_t accel_get_x_value(accelerometer_driver_t* accel_driver, uint16_t buffer_index);
float32_t accel_get_y_value(accelerometer_driver_t* accel_driver, uint16_t buffer_index);
float32_t accel_get_z_value(accelerometer_driver_t* accel_driver, uint16_t buffer_index);

#endif /* APPLICATION_USER_BSP_YETIMOTE_ACCELEROMETER_ACCELEROMETER_DRIVER_H_ */
