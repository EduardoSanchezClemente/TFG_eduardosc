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
 * accelerometer_driver.c
 *
 *  Created on: 16 de ene. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file accelerometer_driver.c
 */

#include "accelerometer_driver.h"
#include "lis3dh_driver.h"
#include "lis3dsh_driver.h"

/**
 *
 * @param accel_type
 * @return
 */
accelerometer_driver_t* new_accelerometer_driver(accelerometer_type_t accel_type){

	accelerometer_driver_t* accel_driver;

	switch(accel_type){
	case LIS3DH:
		accel_driver = pvPortMalloc(sizeof(accelerometer_driver_t));
		accel_driver->accel_funcs = lis3dh_driver_func;
		accel_driver->accel_type = accel_type;
		if(accel_driver->accel_funcs.accel_driver_init(accel_driver) != RET_OK){		//Initialize the driver
			vPortFree(accel_driver);
			accel_driver = NULL;
		}
		break;
	case LIS3DSH:
		accel_driver = pvPortMalloc(sizeof(accelerometer_driver_t));
		accel_driver->accel_funcs = lis3dsh_driver_func;
		accel_driver->accel_type = accel_type;
		if(accel_driver->accel_funcs.accel_driver_init(accel_driver) != RET_OK){		//Initialize the driver
			vPortFree(accel_driver);
			accel_driver = NULL;
		}
		break;
	case MMA8652:
		accel_driver = pvPortMalloc(sizeof(accelerometer_driver_t));
		accel_driver->accel_type = accel_type;
//		accel_driver->accel_funcs = mma8652_driver_func;	//Comentado de momento ya que no existe el driver
//		if(accel_driver->accel_funcs.accel_driver_init(accel_driver) != RET_OK){
//			vPortFree(accel_driver);
//			accel_driver = NULL;
//		}
		break;
	default:
		return NULL;
		break;
	}

	accel_driver->sample_num = DEFAULT_SAMPLE_NUM;
	accel_driver->xAxis = (float32_t*) pvPortMalloc(DEFAULT_SAMPLE_NUM*sizeof(float32_t));
	accel_driver->yAxis = (float32_t*) pvPortMalloc(DEFAULT_SAMPLE_NUM*sizeof(float32_t));
	accel_driver->zAxis = (float32_t*) pvPortMalloc(DEFAULT_SAMPLE_NUM*sizeof(float32_t));

	return accel_driver;
}

/**
 *
 * @param accel_driver
 * @return
 */
retval_t delete_accelerometer_driver(accelerometer_driver_t* accel_driver){

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	if(accel_driver->accel_funcs.accel_driver_close(accel_driver) != RET_OK){		//Close
		return RET_ERROR;
	}

	vPortFree(accel_driver->xAxis);
	vPortFree(accel_driver->yAxis);
	vPortFree(accel_driver->zAxis);

	vPortFree(accel_driver);


	return RET_OK;

}

/**
 *
 * @param accel_driver
 * @param sample_number
 * @return
 */
retval_t accel_set_sample_number(accelerometer_driver_t* accel_driver, uint16_t sample_number){
	if (accel_driver == NULL){
		return RET_ERROR;
	}

	if(sample_number > MAX_SAMPLE_NUMBER){
		return RET_ERROR;
	}
	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	vPortFree(accel_driver->xAxis);
	vPortFree(accel_driver->yAxis);
	vPortFree(accel_driver->zAxis);

	accel_driver->sample_num = sample_number;
	accel_driver->xAxis = (float32_t*) pvPortMalloc(sample_number*sizeof(float32_t));
	accel_driver->yAxis = (float32_t*) pvPortMalloc(sample_number*sizeof(float32_t));
	accel_driver->zAxis = (float32_t*) pvPortMalloc(sample_number*sizeof(float32_t));

	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 *
 * @param accel_driver
 * @return
 */
uint16_t accel_get_sample_number(accelerometer_driver_t* accel_driver){
	if (accel_driver == NULL){
		return NO_ACEL_TYPE;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	uint16_t sample_number = accel_driver->sample_num ;
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return sample_number;
}

/**
 *
 * @param accel_driver
 * @return
 */
accelerometer_type_t accel_get_type(accelerometer_driver_t* accel_driver){
	if (accel_driver == NULL){
		return NO_ACEL_TYPE;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	accelerometer_type_t accel_type = accel_driver->accel_type;
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return accel_type;
}

/**
 *
 * @param accel_driver
 * @return
 */
accelerometer_hw_state_t accel_get_run_mode(accelerometer_driver_t* accel_driver){
	if (accel_driver == NULL){
		return ACEL_STATE_POWER_DOWN;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	accelerometer_hw_state_t accel_hw_state = accel_driver->accel_hw_state;
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return accel_hw_state;
}

/**
 *
 * @param accel_driver
 * @return
 */
uint16_t accel_get_odr(accelerometer_driver_t* accel_driver){
	if (accel_driver == NULL){
		return 0;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	uint16_t odr = accel_driver->current_odr;
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return odr;
}

/**
 *
 * @param accel_driver
 * @return
 */
uint16_t accel_get_scale(accelerometer_driver_t* accel_driver){
	if (accel_driver == NULL){
		return 0;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	uint16_t scale = accel_driver->full_scale;
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return scale;
}

/**
 *
 * @param accel_driver
 * @return
 */
uint8_t accel_get_enabled_axis(accelerometer_driver_t* accel_driver){
	if (accel_driver == NULL){
		return 0;
	}
	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	uint8_t axis = accel_driver->enabled_axis;
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return axis;
}

/**
 *
 * @param accel_driver
 * @param buffer_index
 * @return
 */
float32_t accel_get_x_value(accelerometer_driver_t* accel_driver, uint16_t buffer_index){
	if (accel_driver == NULL){
		return 0;
	}
	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	if(buffer_index >= accel_driver->sample_num){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return 0;
	}
	float32_t value = accel_driver->xAxis[buffer_index];
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return value;
}

/**
 *
 * @param accel_driver
 * @param buffer_index
 * @return
 */
float32_t accel_get_y_value(accelerometer_driver_t* accel_driver, uint16_t buffer_index){
	if (accel_driver == NULL){
		return 0;
	}
	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	if(buffer_index >= accel_driver->sample_num){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return 0;
	}
	float32_t value = accel_driver->yAxis[buffer_index];
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return value;
}

/**
 *
 * @param accel_driver
 * @param buffer_index
 * @return
 */
float32_t accel_get_z_value(accelerometer_driver_t* accel_driver, uint16_t buffer_index){
	if (accel_driver == NULL){
		return 0;
	}
	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);
	if(buffer_index >= accel_driver->sample_num){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return 0;
	}
	float32_t value = accel_driver->zAxis[buffer_index];
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return value;
}

