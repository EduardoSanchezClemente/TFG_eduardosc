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
 * lis3dsh_driver.c
 *
 *  Created on: 16 de ene. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file lis3dsh_driver.c
 */


#include "lis3dsh_driver.h"

osMutexDef (lis3dsh_mutex);
extern i2c_driver_t* i2c1_driver;
int16_t lis3dsh_i2c_port_id;

retval_t lis3dsh_init(accelerometer_driver_t* accel_driver);
retval_t lis3dsh_close(accelerometer_driver_t* accel_driver);
retval_t lis3dsh_normal_run_mode(accelerometer_driver_t* accel_driver);
retval_t lis3dsh_low_power_mode(accelerometer_driver_t* accel_driver);
retval_t lis3dsh_power_down_mode(accelerometer_driver_t* accel_driver);
retval_t lis3dsh_set_odr(accelerometer_driver_t* accel_driver, uint16_t odr);
retval_t lis3dsh_set_scale(accelerometer_driver_t* accel_driver, uint16_t new_scale);
retval_t lis3dsh_enable_axis(accelerometer_driver_t* accel_driver, uint8_t axis);
retval_t lis3dsh_disable_axis(accelerometer_driver_t* accel_driver, uint8_t axis);
retval_t lis3dsh_config_hp_filter(accelerometer_driver_t* accel_driver, uint8_t enable, hp_cutoff_t filter_cutoff);
retval_t lis3dsh_read_single_sample(accelerometer_driver_t* accel_driver);
#if	ACEL_USE_INT_PIN
retval_t lis3dsh_read_stream_samples_it(accelerometer_driver_t* accel_driver);
retval_t lis3dsh_interrupt_callback(void);
#endif
retval_t lis3dsh_read_stream_samples_poll(accelerometer_driver_t* accel_driver);

accelerometer_driver_funcs_t lis3dsh_driver_func = {
		lis3dsh_init,
		lis3dsh_close,
		lis3dsh_normal_run_mode,
		lis3dsh_low_power_mode,
		lis3dsh_power_down_mode,
		lis3dsh_set_odr,
		lis3dsh_set_scale,
		lis3dsh_enable_axis,
		lis3dsh_disable_axis,
		lis3dsh_read_single_sample,
#if	ACEL_USE_INT_PIN
		lis3dsh_read_stream_samples_it,
		lis3dsh_interrupt_callback,
#endif
		lis3dsh_read_stream_samples_poll,
};



/**
 * @brief					Initialize the lis3dsh driver with a default config: 1250Hz ODR, Normal Mode, Fullscale +-2g, xyz enable, fifo stream mode
 * @param accel_driver		Acelerometer driver object
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_init(accelerometer_driver_t* accel_driver){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}
	accel_driver->accelerometer_mutex_id = osMutexCreate (osMutex (lis3dsh_mutex));

	if(lis3dsh_i2c_port_id != -1){
		i2c1_driver->i2c_funcs.i2c_close(i2c1_driver, lis3dsh_i2c_port_id);
		lis3dsh_i2c_port_id = -1;
	}
	lis3dsh_i2c_port_id = i2c1_driver->i2c_funcs.i2c_open(i2c1_driver, LIS3DSH_MEMS_I2C_ADDRESS, I2C_PORT_MASTER);//Lo primero es abrir el puerto I2C a utilizar

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if(lis3dsh_i2c_port_id == -1){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	//Default Config
	if((response = LIS3DSH_SetMode(LIS3DSH_NORMAL)) != MEMS_SUCCESS){			//NORMAL MODE
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;	//OK
	}
	if((response =LIS3DSH_SetODR(LIS3DSH_ODR_1600Hz)) != MEMS_SUCCESS){		//ODR 1600HZ
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;	//OK
	}

	if((response = LIS3DSH_SetFullScale(LIS3DSH_FULLSCALE_2)) != MEMS_SUCCESS){	//FULLSCALE_+-2g
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;	//OK
	}
	if((response = LIS3DSH_SetAxis(LIS3DSH_X_ENABLE | LIS3DSH_Y_ENABLE | LIS3DSH_Z_ENABLE)) != MEMS_SUCCESS){	//XYZ ENABLE
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;	//OK
	}
	if((response = LIS3DSH_FIFOModeEnable(LIS3DSH_FIFO_STREAM_MODE)) != MEMS_SUCCESS){	//FIFO STREAM MODE
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;	//OK
	}

#if ACEL_USE_INT_PIN
	if((response = LIS3DSH_SetInt1Pin(LIS3DSH_WTM_ON_INT1_ENABLE)) != MEMS_SUCCESS){	//SET INTERRUPT PIN
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((LIS3DSH_SetWaterMark(FIFO_THREESHOLD)) != MEMS_SUCCESS){	//SET FIFO THREESHOLD
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
#endif

	accel_driver->available_odr = pvPortMalloc(MAX_ODR_AVAILABLE*sizeof(uint16_t));

	accel_driver->num_available_odr = MAX_ODR_AVAILABLE;
	accel_driver->available_odr[0] = 3;			//SET THE ODR available in normal mode
	accel_driver->available_odr[1] = 6;
	accel_driver->available_odr[2] = 12;
	accel_driver->available_odr[3] = 25;
	accel_driver->available_odr[4] = 50;
	accel_driver->available_odr[5] = 100;
	accel_driver->available_odr[6] = 400;
	accel_driver->available_odr[7] = 800;
	accel_driver->available_odr[8] = 1600;
	accel_driver->full_scale = 2;
	accel_driver->accel_hw_state = ACEL_STATE_NORMAL;
	accel_driver->enabled_axis = ACEL_AXIS_X | ACEL_AXIS_Y | ACEL_AXIS_Z;
	accel_driver->resolution = ACEL_2G_RESOLUTION;
	accel_driver->accel_hp_filter_state = HPF_STATE_DISABLED;
	accel_driver->current_odr = 1600;
	accel_driver->hp_cutoff_factor = accel_driver->current_odr;
	accel_driver->hp_cutoff_freq = (float32_t) accel_driver->current_odr/accel_driver->hp_cutoff_factor;
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 * @brief					Close the lis3dsh driver. Set it to POWER_DOWN_MODE
 * @param accel_driver		Acelerometer driver object
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_close(accelerometer_driver_t* accel_driver){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if((response = LIS3DSH_SetMode(LIS3DSH_POWER_DOWN)) != MEMS_SUCCESS){			//POWER_DOWN MODE
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}


	vPortFree(accel_driver->available_odr);
	accel_driver->available_odr = NULL;

	i2c1_driver->i2c_funcs.i2c_close(i2c1_driver, lis3dsh_i2c_port_id);
	lis3dsh_i2c_port_id = -1;
	osMutexRelease(accel_driver->accelerometer_mutex_id);

	osMutexDelete(accel_driver->accelerometer_mutex_id);

	return RET_OK;
}

/**
 * @brief 					Set the lis3dsh to normal mode
 * @param accel_driver		Acelerometer driver object
 * @return					Status [RET_OK, RET_ERROR]
 * 							When changed the mode the set_odr() function must be called.
 */
retval_t lis3dsh_normal_run_mode(accelerometer_driver_t* accel_driver){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if(accel_driver->accel_hw_state != ACEL_STATE_NORMAL){
		if((response = LIS3DSH_SetMode(LIS3DSH_NORMAL)) != MEMS_SUCCESS){
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->num_available_odr = MAX_ODR_AVAILABLE;
		accel_driver->accel_hw_state = ACEL_STATE_NORMAL;
		accel_driver->current_odr = 800;
		accel_driver->hp_cutoff_factor = accel_driver->current_odr;
	}
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 * @brief					Set the lis3dsh to low power mode. This mode is not supported on this accelerometer. Do nothing
 * @param accel_driver		Acelerometer driver object
 * @return 					Status [RET_OK, RET_ERROR] Always RET_ERROR
 */
retval_t lis3dsh_low_power_mode(accelerometer_driver_t* accel_driver){

	return RET_ERROR;
}

/**
 * @brief						Set the lis3dsh to power down mode. No signal is captured in this mode
 * @param 	accel_driver			Acelerometer driver object
 * @return						Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_power_down_mode(accelerometer_driver_t* accel_driver){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if(accel_driver->accel_hw_state != ACEL_STATE_POWER_DOWN){
		if((response = LIS3DSH_SetMode(LIS3DSH_POWER_DOWN)) != MEMS_SUCCESS){
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->num_available_odr = 0;
		accel_driver->accel_hw_state = ACEL_STATE_POWER_DOWN;
	}
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 * @brief					Sets the ODR of the accelerometer
 * @param accel_driver		Acelerometer driver object
 * @param odr				New ODR of the accelerometer
 * @return					Status [RET_OK, RET_ERROR]
 * @note					If the state accelerometer state is power down, no ODR setting is allowed
 */
retval_t lis3dsh_set_odr(accelerometer_driver_t* accel_driver, uint16_t odr){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if(accel_driver->accel_hw_state != ACEL_STATE_POWER_DOWN){
		uint16_t i;
		for(i=0; i<accel_driver->num_available_odr; i++){
			if(accel_driver->available_odr[i] == odr){
				break;
			}
		}
		if (i == accel_driver->num_available_odr){
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;	//Wrong ODR used
		}
		accel_driver->current_odr = odr;
		accel_driver->hp_cutoff_freq = (float32_t) accel_driver->current_odr/accel_driver->hp_cutoff_factor;
		switch (i){
			case 0:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_3_125Hz)) != MEMS_SUCCESS){		//ODR 1HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 1:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_6_25Hz)) != MEMS_SUCCESS){		//ODR 10HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 2:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_12_5Hz)) != MEMS_SUCCESS){		//ODR 25HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 3:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_25Hz)) != MEMS_SUCCESS){		//ODR 50HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 4:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_50Hz)) != MEMS_SUCCESS){		//ODR 100HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 5:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_100Hz)) != MEMS_SUCCESS){		//ODR 200HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 6:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_400Hz)) != MEMS_SUCCESS){		//ODR 400HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 7:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_800Hz)) != MEMS_SUCCESS){		//ODR 800HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 8:
				if((response =LIS3DSH_SetODR(LIS3DSH_ODR_1600Hz)) != MEMS_SUCCESS){		//ODR 1600HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			default:
				osMutexRelease(accel_driver->accelerometer_mutex_id);
				return RET_ERROR;
				break;
		}
	}
	else{
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}
/**
 * @brief					Sets the scale of the accelerometer. Allowed values: +-2g, +-4g, +-8g, +-16g
 * @param accel_driver		Acelerometer driver object
 * @param new_scale			New scale value. It only accepts 2, 4, 8 and 16
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_set_scale(accelerometer_driver_t* accel_driver, uint16_t new_scale){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	switch (new_scale){
	case 2:
		if((response = LIS3DSH_SetFullScale(LIS3DSH_FULLSCALE_2)) != MEMS_SUCCESS){	//FULLSCALE_+-2g
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->resolution = ACEL_2G_RESOLUTION;
		accel_driver->full_scale = 2;
		break;
	case 4:
		if((response = LIS3DSH_SetFullScale(LIS3DSH_FULLSCALE_4)) != MEMS_SUCCESS){	//FULLSCALE_+-4g
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->resolution = ACEL_4G_RESOLUTION;
		accel_driver->full_scale = 4;
		break;
	case 8:
		if((response = LIS3DSH_SetFullScale(LIS3DSH_FULLSCALE_8)) != MEMS_SUCCESS){	//FULLSCALE_+-8g
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->resolution = ACEL_8G_RESOLUTION;
		accel_driver->full_scale = 8;
		break;
	case 16:
		if((response = LIS3DSH_SetFullScale(LIS3DSH_FULLSCALE_16)) != MEMS_SUCCESS){	//FULLSCALE_+-16g
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->resolution = ACEL_16G_RESOLUTION;
		accel_driver->full_scale = 16;
		break;
	default:
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
		break;
	}
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 * @brief					Enable the output of the specified axis
 * @param accel_driver		Acelerometer driver object
 * @param axis				Axis to be activated.Accepted values [ACEL_AXIS_X, ACEL_AXIS_Y, ACEL_AXIS_Z]. They may be ORed to activate all or some together
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_enable_axis(accelerometer_driver_t* accel_driver, uint8_t axis){
	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	accel_driver->enabled_axis |= axis;
	if((response = LIS3DSH_SetAxis(accel_driver->enabled_axis)) != MEMS_SUCCESS){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}

	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 * @brief					Disable the output of the specified axis
 * @param accel_driver		Acelerometer driver object
 * @param axis				Axis to be disabled. Accepted values [ACEL_AXIS_X, ACEL_AXIS_Y, ACEL_AXIS_Z]. They may be ORed to disable all or some together
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_disable_axis(accelerometer_driver_t* accel_driver, uint8_t axis){
	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if (axis > 0x07){		//Wrong input axis value
		return RET_ERROR;
	}

	accel_driver->enabled_axis = ((~axis) & accel_driver->enabled_axis);
	if((response = LIS3DSH_SetAxis(accel_driver->enabled_axis)) != MEMS_SUCCESS){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}

	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}


/**
 * @brief					Set the High Pass Filter configuration. This function is not allowed on LIS3DSH
 * @param accel_driver		Acelerometer driver object
 * @param filter_state		Set the high pass filter. [HPF_STATE_ENABLED, HPF_STATE_DISABLED]
 * @param filter_cutoff		Set the cutoff frequency [HP_CUTOFF_LOWEST, HP_CUTOFF_LOW, HP_CUTOFF_HIGH, HP_CUTOFF_HIGHEST]. The cutoff frequency is the ODR divided by 400, 200, 100 or 50
 * @return					Status [RET_OK, RET_ERROR] Always return RET_ERROR
 */
retval_t lis3dsh_config_hp_filter(accelerometer_driver_t* accel_driver, accelerometer_hp_filter_state_t filter_state, hp_cutoff_t filter_cutoff){

	return RET_ERROR;
}
/**
 * @brief					Read a single sample from the accelerometer. Only read the enabled axis.  Value are stored in the first buffer value of the xAxis, yAxis and zAxis variables
 * @param accel_driver		Acelerometer driver object
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_read_single_sample(accelerometer_driver_t* accel_driver){
	status_t response;
	AxesRaw_t readData;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if (accel_driver->accel_hw_state == ACEL_STATE_POWER_DOWN){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}

	if((response = LIS3DSH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}

	if(accel_driver->sample_num > 0){
		if(accel_driver->enabled_axis & ACEL_AXIS_X){
			accel_driver->xAxis[0] = (float32_t) readData.AXIS_X * accel_driver->resolution;
		}
		if(accel_driver->enabled_axis & ACEL_AXIS_Y){
			accel_driver->yAxis[0] = (float32_t) readData.AXIS_Y * accel_driver->resolution;
		}
		if(accel_driver->enabled_axis & ACEL_AXIS_Z){
			accel_driver->zAxis[0] = (float32_t) readData.AXIS_Z * accel_driver->resolution;
		}
	}
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

#if ACEL_USE_INT_PIN
osThreadId callingThread = NULL;

/**
 * @brief				Reads the specified number of samples from the accelerometer for the enabled axis. Uses the accelerometer FIFO interrupt pin.
 * @param accel_driver	Acelerometer driver object
 * @param numSamples	Number of samples to read
 * @param xbuffer		X Axis Output Buffer
 * @param ybuffer		Y Axis Output Buffer
 * @param zbuffer		Z Axis Output Buffer
 * @return				Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_read_stream_samples_it(accelerometer_driver_t* accel_driver){

	status_t response;
	AxesRaw_t readData;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if (accel_driver->accel_hw_state == ACEL_STATE_POWER_DOWN){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}

	uint16_t i;
	if(accel_driver->sample_num < 32) {								//In this case no interrupt management is necessary
		uint8_t fifo_samples;
		LIS3DSH_GetFifoSourceFSS(&fifo_samples);
		while(fifo_samples < accel_driver->sample_num){
			osDelay(1);
			LIS3DSH_GetFifoSourceFSS(&fifo_samples);			//Wait till the FIFO has the specified sample number
		}

		for(i=0; i<accel_driver->sample_num; i++){

			if((response = LIS3DSH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
				osMutexRelease(accel_driver->accelerometer_mutex_id);
				return RET_ERROR;
			}

			if(accel_driver->enabled_axis & ACEL_AXIS_X){
				accel_driver->xAxis[i] = (float32_t) readData.AXIS_X * accel_driver->resolution;
			}
			if(accel_driver->enabled_axis & ACEL_AXIS_Y){
				accel_driver->yAxis[i] = (float32_t) readData.AXIS_Y * accel_driver->resolution;
			}
			if(accel_driver->enabled_axis & ACEL_AXIS_Z){
				accel_driver->zAxis[i] = (float32_t) readData.AXIS_Z * accel_driver->resolution;
			}

		}
	}

	else{
		callingThread = osThreadGetId();
		uint16_t readSamples = 0;
		uint16_t remaingSamples = accel_driver->sample_num - readSamples;
		uint8_t fifo_samples;
		while(readSamples < accel_driver->sample_num){

			remaingSamples = accel_driver->sample_num - readSamples;
			LIS3DSH_GetFifoSourceFSS(&fifo_samples);

			if((remaingSamples) < FIFO_THREESHOLD){		//In this case the remaining samples to be read are lower to the FIFO Threeshold. Active wait till get this samples
				while(fifo_samples < remaingSamples){
					osDelay(1);
					LIS3DSH_GetFifoSourceFSS(&fifo_samples);			//Wait till the FIFO has the specified sample number
				}

				for(i=0; i<remaingSamples; i++){

					if((response = LIS3DSH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
						callingThread = NULL;
						osMutexRelease(accel_driver->accelerometer_mutex_id);
						return RET_ERROR;
					}

					if(accel_driver->enabled_axis & ACEL_AXIS_X){
						accel_driver->xAxis[i] = (float32_t) readData.AXIS_X * accel_driver->resolution;
					}
					if(accel_driver->enabled_axis & ACEL_AXIS_Y){
						accel_driver->yAxis[i] = (float32_t) readData.AXIS_Y * accel_driver->resolution;
					}
					if(accel_driver->enabled_axis & ACEL_AXIS_Z){
						accel_driver->zAxis[i] = (float32_t) readData.AXIS_Z * accel_driver->resolution;
					}
					readSamples++;
				}
				break;
			}

			while(fifo_samples < FIFO_THREESHOLD){			//In case of fifo samples lower to threeshold wait till the interrupt happens
				osThreadSuspend(callingThread);
				LIS3DSH_GetFifoSourceFSS(&fifo_samples);
			}

			for(i=0; i<FIFO_THREESHOLD; i++){

				if((response = LIS3DSH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
					callingThread = NULL;
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}

				if(accel_driver->enabled_axis & ACEL_AXIS_X){
					accel_driver->xAxis[i] = (float32_t) readData.AXIS_X * accel_driver->resolution;
				}
				if(accel_driver->enabled_axis & ACEL_AXIS_Y){
					accel_driver->yAxis[i] = (float32_t) readData.AXIS_Y * accel_driver->resolution;
				}
				if(accel_driver->enabled_axis & ACEL_AXIS_Z){
					accel_driver->zAxis[i] = (float32_t) readData.AXIS_Z * accel_driver->resolution;
				}
				readSamples++;
			}
		}

		callingThread = NULL;
	}


	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;

}

/**
 * @brief	Interrupt routine for the accelerometer FIFO threeshold event
 * @return	Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_interrupt_callback(void){
	if(callingThread != NULL){
		osThreadResume(callingThread);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
#endif


/**
 * @brief				Reads the specified number of samples from the accelerometer for the enabled axis. Uses poll mode. Use this function when
 * 						the interrupt pin is not available.
 * @param accel_driver	Acelerometer driver object
 * @param numSamples	Number of samples to read
 * @param xbuffer		X Axis Output Buffer
 * @param ybuffer		Y Axis Output Buffer
 * @param zbuffer		Z Axis Output Buffer
 * @return				Status [RET_OK, RET_ERROR]
 */
retval_t lis3dsh_read_stream_samples_poll(accelerometer_driver_t* accel_driver){

	status_t response;
	AxesRaw_t readData;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if (accel_driver->accel_hw_state == ACEL_STATE_POWER_DOWN){
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
	}

	uint16_t i;
	if(accel_driver->sample_num < 32) {								//In this case no special management is necessary
		uint8_t fifo_samples;
		LIS3DSH_GetFifoSourceFSS(&fifo_samples);
		while(fifo_samples < accel_driver->sample_num){
			osDelay(1);
			LIS3DSH_GetFifoSourceFSS(&fifo_samples);			//Wait till the FIFO has the specified sample number
		}

		for(i=0; i<accel_driver->sample_num; i++){

			if((response = LIS3DSH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
				osMutexRelease(accel_driver->accelerometer_mutex_id);
				return RET_ERROR;
			}

			if(accel_driver->enabled_axis & ACEL_AXIS_X){
				accel_driver->xAxis[i] = (float32_t) readData.AXIS_X * accel_driver->resolution;
			}
			if(accel_driver->enabled_axis & ACEL_AXIS_Y){
				accel_driver->yAxis[i] = (float32_t) readData.AXIS_Y * accel_driver->resolution;
			}
			if(accel_driver->enabled_axis & ACEL_AXIS_Z){
				accel_driver->zAxis[i] = (float32_t) readData.AXIS_Z * accel_driver->resolution;
			}

		}
	}

	else{
		uint16_t readSamples = 0;
		uint16_t remaingSamples = accel_driver->sample_num - readSamples;
		uint8_t fifo_samples;
		while(readSamples < accel_driver->sample_num){

			remaingSamples = accel_driver->sample_num - readSamples;
			LIS3DSH_GetFifoSourceFSS(&fifo_samples);

			if((remaingSamples) < FIFO_THREESHOLD){		//In this case the remaining samples to be read are lower to the FIFO Threeshold. Active wait till get this samples
				while(fifo_samples < remaingSamples){
					osDelay(2);
					LIS3DSH_GetFifoSourceFSS(&fifo_samples);			//Wait till the FIFO has the specified sample number
				}

				for(i=0; i<remaingSamples; i++){

					if((response = LIS3DSH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
						osMutexRelease(accel_driver->accelerometer_mutex_id);
						return RET_ERROR;
					}

					if(accel_driver->enabled_axis & ACEL_AXIS_X){
						accel_driver->xAxis[readSamples] = (float32_t) readData.AXIS_X * accel_driver->resolution;
					}
					if(accel_driver->enabled_axis & ACEL_AXIS_Y){
						accel_driver->yAxis[readSamples] = (float32_t) readData.AXIS_Y * accel_driver->resolution;
					}
					if(accel_driver->enabled_axis & ACEL_AXIS_Z){
						accel_driver->zAxis[readSamples] = (float32_t) readData.AXIS_Z * accel_driver->resolution;
					}
					readSamples++;
				}
				break;
			}

			while(fifo_samples < FIFO_THREESHOLD){			//In case of fifo samples lower to threeshold wait poll for the FIFO_THREESHOLD level
				osDelay(2);
				LIS3DSH_GetFifoSourceFSS(&fifo_samples);
			}

			for(i=0; i<FIFO_THREESHOLD; i++){

				if((response = LIS3DSH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}

				if(accel_driver->enabled_axis & ACEL_AXIS_X){
					accel_driver->xAxis[readSamples] = (float32_t) readData.AXIS_X * accel_driver->resolution;
				}
				if(accel_driver->enabled_axis & ACEL_AXIS_Y){
					accel_driver->yAxis[readSamples] = (float32_t) readData.AXIS_Y * accel_driver->resolution;
				}
				if(accel_driver->enabled_axis & ACEL_AXIS_Z){
					accel_driver->zAxis[readSamples] = (float32_t) readData.AXIS_Z * accel_driver->resolution;
				}
				readSamples++;
			}
		}


	}


	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;

}

//uint16_t lis3dsh_get_hp_filter_state(accelerometer_driver_t* accel_driver);
//float32_t lis3dsh_get_hp_filter_cutoff_freq(accelerometer_driver_t* accel_driver);

