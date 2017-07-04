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
 * lis3dh_driver.c
 *
 *  Created on: 16 de ene. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file lis3dh_driver.c
 */


#include "lis3dh_driver.h"

osMutexDef (lis3dh_mutex);
extern i2c_driver_t* i2c1_driver;
int16_t lis3dh_i2c_port_id;

retval_t lis3dh_init(accelerometer_driver_t* accel_driver);
retval_t lis3dh_close(accelerometer_driver_t* accel_driver);
retval_t lis3dh_normal_run_mode(accelerometer_driver_t* accel_driver);
retval_t lis3dh_low_power_mode(accelerometer_driver_t* accel_driver);
retval_t lis3dh_power_down_mode(accelerometer_driver_t* accel_driver);
retval_t lis3dh_set_odr(accelerometer_driver_t* accel_driver, uint16_t odr);
retval_t lis3dh_set_scale(accelerometer_driver_t* accel_driver, uint16_t new_scale);
retval_t lis3dh_enable_axis(accelerometer_driver_t* accel_driver, uint8_t axis);
retval_t lis3dh_disable_axis(accelerometer_driver_t* accel_driver, uint8_t axis);
retval_t lis3dh_config_hp_filter(accelerometer_driver_t* accel_driver, uint8_t enable, hp_cutoff_t filter_cutoff);
retval_t lis3dh_read_single_sample(accelerometer_driver_t* accel_driver);
#if	ACEL_USE_INT_PIN
retval_t lis3dh_read_stream_samples_it(accelerometer_driver_t* accel_driver);
retval_t lis3dh_interrupt_callback(void);
#endif
retval_t lis3dh_read_stream_samples_poll(accelerometer_driver_t* accel_driver);

//uint16_t lis3dh_get_hp_filter_state(accelerometer_driver_t* accel_driver);
//float32_t lis3dh_get_hp_filter_cutoff_freq(accelerometer_driver_t* accel_driver);

accelerometer_driver_funcs_t lis3dh_driver_func = {
		lis3dh_init,
		lis3dh_close,
		lis3dh_normal_run_mode,
		lis3dh_low_power_mode,
		lis3dh_power_down_mode,
		lis3dh_set_odr,
		lis3dh_set_scale,
		lis3dh_enable_axis,
		lis3dh_disable_axis,
		lis3dh_read_single_sample,
#if	ACEL_USE_INT_PIN
		lis3dh_read_stream_samples_it,
		lis3dh_interrupt_callback,
#endif
		lis3dh_read_stream_samples_poll,
};



/**
 * @brief					Initialize the lis3dh driver with a default config: 1250Hz ODR, Normal Mode, Fullscale +-2g, xyz enable, fifo stream mode
 * @param accel_driver		Acelerometer driver object
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dh_init(accelerometer_driver_t* accel_driver){

	status_t response;

	//Default Config
	accel_driver->accelerometer_mutex_id = osMutexCreate (osMutex (lis3dh_mutex));

	if (accel_driver == NULL){
		return RET_ERROR;
	}
	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if(lis3dh_i2c_port_id != -1){
		i2c1_driver->i2c_funcs.i2c_close(i2c1_driver, lis3dh_i2c_port_id);
		lis3dh_i2c_port_id = -1;
	}
	lis3dh_i2c_port_id = i2c1_driver->i2c_funcs.i2c_open(i2c1_driver, LIS3DH_MEMS_I2C_ADDRESS, I2C_PORT_MASTER);//Lo primero es abrir el puerto I2C a utilizar

	if(lis3dh_i2c_port_id == -1){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}

	if((response =LIS3DH_SetODR(LIS3DH_ODR_1344Hz_NP_5367HZ_LP)) != MEMS_SUCCESS){		//ODR 1250HZ
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((response = LIS3DH_SetMode(LIS3DH_NORMAL)) != MEMS_SUCCESS){			//NORMAL MODE
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2)) != MEMS_SUCCESS){	//FULLSCALE_+-2g
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((response = LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE)) != MEMS_SUCCESS){	//XYZ ENABLE
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((response = LIS3DH_FIFOModeEnable(LIS3DH_FIFO_STREAM_MODE)) != MEMS_SUCCESS){	//FIFO STREAM MODE
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((response = LIS3DH_SetHPFMode(LIS3DH_HPM_NORMAL_MODE)) != MEMS_SUCCESS){	//High pass Filter normal mode
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((response = LIS3DH_SetFilterDataSel(MEMS_RESET)) != MEMS_SUCCESS){	//High pass Filter initially disabled
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((response = LIS3DH_SetHPFCutOFF(3)) != MEMS_SUCCESS){	//High pass Filter Cut off Frequency 11. This means ODR/400
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}


#if ACEL_USE_INT_PIN
	if((response = LIS3DH_SetInt1Pin(LIS3DH_WTM_ON_INT1_ENABLE)) != MEMS_SUCCESS){	//SET INTERRUPT PIN
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
	if((LIS3DH_SetWaterMark(FIFO_THREESHOLD)) != MEMS_SUCCESS){	//SET FIFO THREESHOLD
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}
#endif

	accel_driver->available_odr = pvPortMalloc(MAX_ODR_AVAILABLE*sizeof(uint16_t));

	accel_driver->num_available_odr = NORMAL_MODE_AVAILABLE_ODR;
	accel_driver->available_odr[0] = 1;			//SET THE ODR available in normal mode
	accel_driver->available_odr[1] = 10;
	accel_driver->available_odr[2] = 25;
	accel_driver->available_odr[3] = 50;
	accel_driver->available_odr[4] = 100;
	accel_driver->available_odr[5] = 200;
	accel_driver->available_odr[6] = 400;
	accel_driver->available_odr[7] = 1344;
	accel_driver->full_scale = 2;
	accel_driver->accel_hw_state = ACEL_STATE_NORMAL;
	accel_driver->enabled_axis = ACEL_AXIS_X | ACEL_AXIS_Y | ACEL_AXIS_Z;
	accel_driver->resolution = ACEL_2G_RESOLUTION;
	accel_driver->accel_hp_filter_state = HPF_STATE_DISABLED;
	accel_driver->current_odr = 1344;
	accel_driver->hp_cutoff_factor = 400;
	accel_driver->hp_cutoff_freq = (float32_t) accel_driver->current_odr/accel_driver->hp_cutoff_factor;
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 * @brief					Close the lis3dh driver. Set it to POWER_DOWN_MODE
 * @param accel_driver		Acelerometer driver object
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dh_close(accelerometer_driver_t* accel_driver){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if((response = LIS3DH_SetMode(LIS3DH_POWER_DOWN)) != MEMS_SUCCESS){			//POWER_DOWN MODE
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}

	vPortFree(accel_driver->available_odr);
	accel_driver->available_odr = NULL;

	i2c1_driver->i2c_funcs.i2c_close(i2c1_driver, lis3dh_i2c_port_id);
	lis3dh_i2c_port_id = -1;
	osMutexRelease(accel_driver->accelerometer_mutex_id);

	osMutexDelete(accel_driver->accelerometer_mutex_id);

	return RET_OK;
}

/**
 * @brief 					Set the lis3dh to normal mode
 * @param accel_driver		Acelerometer driver object
 * @return					Status [RET_OK, RET_ERROR]
 * @note					Supported ODR in this mode: 10, 25, 50, 100, 200, 400, 1250 Hz.
 * 							When changed the mode the set_odr() function must be called.
 */
retval_t lis3dh_normal_run_mode(accelerometer_driver_t* accel_driver){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if(accel_driver->accel_hw_state != ACEL_STATE_NORMAL){
		if((response = LIS3DH_SetMode(LIS3DH_NORMAL)) != MEMS_SUCCESS){
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->num_available_odr = NORMAL_MODE_AVAILABLE_ODR;
		accel_driver->available_odr[7] = 1344;						//Only changes one available ODR
		accel_driver->accel_hw_state = ACEL_STATE_NORMAL;
		if(accel_driver->current_odr == 5376){
			accel_driver->current_odr = 1344;
			accel_driver->hp_cutoff_freq = (float32_t) accel_driver->current_odr/accel_driver->hp_cutoff_factor;
		}
	}
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 * @brief					Set the lis3dh to low power mode
 * @param accel_driver		Acelerometer driver object
 * @return 					Status [RET_OK, RET_ERROR]
 * @note					Supported ODR in this mode: 10, 25, 50, 100, 200, 400, 1600, 5000 Hz
 * 							When changed the mode the set_odr() function must be called.
 */
retval_t lis3dh_low_power_mode(accelerometer_driver_t* accel_driver){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if(accel_driver->accel_hw_state != ACEL_STATE_LOW_POWER){
		if((response = LIS3DH_SetMode(LIS3DH_LOW_POWER)) != MEMS_SUCCESS){
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->num_available_odr = LOW_POWER_MODE_AVAILABLE_ODR;
		accel_driver->available_odr[7] = 1600;						//Changes two available ODR
		accel_driver->available_odr[8] = 5376;
		accel_driver->accel_hw_state = ACEL_STATE_LOW_POWER;
		if(accel_driver->current_odr == 1250){
			accel_driver->current_odr = 5376;
			accel_driver->hp_cutoff_freq = (float32_t) accel_driver->current_odr/accel_driver->hp_cutoff_factor;
		}

	}
	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}

/**
 * @brief						Set the lis3dh to power down mode. No signal is captured in this mode
 * @param 	accel_driver			Acelerometer driver object
 * @return						Status [RET_OK, RET_ERROR]
 */
retval_t lis3dh_power_down_mode(accelerometer_driver_t* accel_driver){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if(accel_driver->accel_hw_state != ACEL_STATE_POWER_DOWN){
		if((response = LIS3DH_SetMode(LIS3DH_POWER_DOWN)) != MEMS_SUCCESS){
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
retval_t lis3dh_set_odr(accelerometer_driver_t* accel_driver, uint16_t odr){

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
				if((response =LIS3DH_SetODR(LIS3DH_ODR_1Hz)) != MEMS_SUCCESS){		//ODR 1HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 1:
				if((response =LIS3DH_SetODR(LIS3DH_ODR_10Hz)) != MEMS_SUCCESS){		//ODR 10HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 2:
				if((response =LIS3DH_SetODR(LIS3DH_ODR_25Hz)) != MEMS_SUCCESS){		//ODR 25HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 3:
				if((response =LIS3DH_SetODR(LIS3DH_ODR_50Hz)) != MEMS_SUCCESS){		//ODR 50HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 4:
				if((response =LIS3DH_SetODR(LIS3DH_ODR_100Hz)) != MEMS_SUCCESS){		//ODR 100HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 5:
				if((response =LIS3DH_SetODR(LIS3DH_ODR_200Hz)) != MEMS_SUCCESS){		//ODR 200HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 6:
				if((response =LIS3DH_SetODR(LIS3DH_ODR_400Hz)) != MEMS_SUCCESS){		//ODR 400HZ
					osMutexRelease(accel_driver->accelerometer_mutex_id);
					return RET_ERROR;
				}
				break;
			case 7:
				if(accel_driver->accel_hw_state == ACEL_STATE_LOW_POWER){
					if((response =LIS3DH_SetODR(LIS3DH_ODR_1620Hz_LP)) != MEMS_SUCCESS){		//ODR 1600HZ
						osMutexRelease(accel_driver->accelerometer_mutex_id);
						return RET_ERROR;
					}
				}
				else{
					if((response =LIS3DH_SetODR(LIS3DH_ODR_1344Hz_NP_5367HZ_LP)) != MEMS_SUCCESS){		//ODR 1250HZ
						osMutexRelease(accel_driver->accelerometer_mutex_id);
						return RET_ERROR;
					}
				}
				break;
			case 8:
				if((response =LIS3DH_SetODR(LIS3DH_ODR_1344Hz_NP_5367HZ_LP)) != MEMS_SUCCESS){		//ODR 5000HZ
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
retval_t lis3dh_set_scale(accelerometer_driver_t* accel_driver, uint16_t new_scale){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	switch (new_scale){
	case 2:
		if((response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2)) != MEMS_SUCCESS){	//FULLSCALE_+-2g
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->resolution = ACEL_2G_RESOLUTION;
		accel_driver->full_scale = 2;
		break;
	case 4:
		if((response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_4)) != MEMS_SUCCESS){	//FULLSCALE_+-4g
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->resolution = ACEL_4G_RESOLUTION;
		accel_driver->full_scale = 4;
		break;
	case 8:
		if((response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_8)) != MEMS_SUCCESS){	//FULLSCALE_+-8g
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->resolution = ACEL_8G_RESOLUTION;
		accel_driver->full_scale = 8;
		break;
	case 16:
		if((response = LIS3DH_SetFullScale(LIS3DH_FULLSCALE_16)) != MEMS_SUCCESS){	//FULLSCALE_+-16g
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
retval_t lis3dh_enable_axis(accelerometer_driver_t* accel_driver, uint8_t axis){
	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	accel_driver->enabled_axis |= axis;
	if((response = LIS3DH_SetAxis(accel_driver->enabled_axis)) != MEMS_SUCCESS){
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
retval_t lis3dh_disable_axis(accelerometer_driver_t* accel_driver, uint8_t axis){
	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	if (axis > 0x07){		//Wrong input axis value
		return RET_ERROR;
	}

	accel_driver->enabled_axis = ((~axis) & accel_driver->enabled_axis);
	if((response = LIS3DH_SetAxis(accel_driver->enabled_axis)) != MEMS_SUCCESS){
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
	}

	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;
}


/**
 * @brief					Set the High Pass Filter configuration. Allows enabling or disabling it and setting its cut-off frequency
 * @param accel_driver		Acelerometer driver object
 * @param filter_state		Set the high pass filter. [HPF_STATE_ENABLED, HPF_STATE_DISABLED]
 * @param filter_cutoff		Set the cutoff frequency [HP_CUTOFF_LOWEST, HP_CUTOFF_LOW, HP_CUTOFF_HIGH, HP_CUTOFF_HIGHEST]. The cutoff frequency is the ODR divided by 400, 200, 100 or 50
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dh_config_hp_filter(accelerometer_driver_t* accel_driver, accelerometer_hp_filter_state_t filter_state, hp_cutoff_t filter_cutoff){

	status_t response;

	if (accel_driver == NULL){
		return RET_ERROR;
	}

	osMutexWait(accel_driver->accelerometer_mutex_id, osWaitForever);

	switch (filter_cutoff){
	case  HP_CUTOFF_LOWEST:
		if((response = LIS3DH_SetHPFCutOFF(3)) != MEMS_SUCCESS){	//High pass Filter Cut off Frequency 11. This means ODR/400
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->hp_cutoff_factor = 400;
		break;
	case HP_CUTOFF_LOW:
		if((response = LIS3DH_SetHPFCutOFF(2)) != MEMS_SUCCESS){	//High pass Filter Cut off Frequency 10. This means ODR/200
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->hp_cutoff_factor = 200;
		break;
	case HP_CUTOFF_HIGH:
		if((response = LIS3DH_SetHPFCutOFF(1)) != MEMS_SUCCESS){	//High pass Filter Cut off Frequency 01. This means ODR/100
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->hp_cutoff_factor = 100;
		break;
	case HP_CUTOFF_HIGHEST:
		if((response = LIS3DH_SetHPFCutOFF(0)) != MEMS_SUCCESS){	//High pass Filter Cut off Frequency 00. This means ODR/50
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->hp_cutoff_factor = 50;
		break;
	default:
		osMutexRelease(accel_driver->accelerometer_mutex_id);
		return RET_ERROR;
		break;
	}

	accel_driver->hp_cutoff_freq = (float32_t) accel_driver->current_odr/accel_driver->hp_cutoff_factor;

	switch (filter_state){
	case HPF_STATE_ENABLED:
		if((response = LIS3DH_SetFilterDataSel(MEMS_SET)) != MEMS_SUCCESS){//Enable
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->accel_hp_filter_state = HPF_STATE_ENABLED;
		break;
	case HPF_STATE_DISABLED:
		if((response = LIS3DH_SetFilterDataSel(MEMS_RESET)) != MEMS_SUCCESS){//Disable
			osMutexRelease(accel_driver->accelerometer_mutex_id);
			return RET_ERROR;
		}
		accel_driver->accel_hp_filter_state = HPF_STATE_DISABLED;
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
 * @brief					Read a single sample from the accelerometer. Only read the enabled axis. Value are stored in the first buffer value of the xAxis, yAxis and zAxis variables
 * @param accel_driver		Acelerometer driver object
 * @return					Status [RET_OK, RET_ERROR]
 */
retval_t lis3dh_read_single_sample(accelerometer_driver_t* accel_driver){
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

	if((response = LIS3DH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
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
retval_t lis3dh_read_stream_samples_it(accelerometer_driver_t* accel_driver){

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
		LIS3DH_GetFifoSourceFSS(&fifo_samples);
		while(fifo_samples < accel_driver->sample_num){
			osDelay(1);
			LIS3DH_GetFifoSourceFSS(&fifo_samples);			//Wait till the FIFO has the specified sample number
		}

		for(i=0; i<accel_driver->sample_num; i++){

			if((response = LIS3DH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
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
			LIS3DH_GetFifoSourceFSS(&fifo_samples);

			if((remaingSamples) < FIFO_THREESHOLD){		//In this case the remaining samples to be read are lower to the FIFO Threeshold. Active wait till get this samples
				while(fifo_samples < remaingSamples){
					osDelay(1);
					LIS3DH_GetFifoSourceFSS(&fifo_samples);			//Wait till the FIFO has the specified sample number
				}

				for(i=0; i<remaingSamples; i++){

					if((response = LIS3DH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
						callingThread = NULL;
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

			while(fifo_samples < FIFO_THREESHOLD){			//In case of fifo samples lower to threeshold wait till the interrupt happens
				osThreadSuspend(callingThread);
				LIS3DH_GetFifoSourceFSS(&fifo_samples);
			}

			for(i=0; i<FIFO_THREESHOLD; i++){

				if((response = LIS3DH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
					callingThread = NULL;
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

		callingThread = NULL;
	}


	osMutexRelease(accel_driver->accelerometer_mutex_id);
	return RET_OK;

}

/**
 * @brief	Interrupt routine for the accelerometer FIFO threeshold event
 * @return	Status [RET_OK, RET_ERROR]
 */
retval_t lis3dh_interrupt_callback(void){
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
retval_t lis3dh_read_stream_samples_poll(accelerometer_driver_t* accel_driver){

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
		LIS3DH_GetFifoSourceFSS(&fifo_samples);
		while(fifo_samples < accel_driver->sample_num){
			osDelay(1);
			LIS3DH_GetFifoSourceFSS(&fifo_samples);			//Wait till the FIFO has the specified sample number
		}

		for(i=0; i<accel_driver->sample_num; i++){

			if((response = LIS3DH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
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
			LIS3DH_GetFifoSourceFSS(&fifo_samples);

			if((remaingSamples) < FIFO_THREESHOLD){		//In this case the remaining samples to be read are lower to the FIFO Threeshold. Active wait till get this samples
				while(fifo_samples < remaingSamples){
					osDelay(2);
					LIS3DH_GetFifoSourceFSS(&fifo_samples);			//Wait till the FIFO has the specified sample number
				}

				for(i=0; i<remaingSamples; i++){

					if((response = LIS3DH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
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
				LIS3DH_GetFifoSourceFSS(&fifo_samples);
			}

			for(i=0; i<FIFO_THREESHOLD; i++){

				if((response = LIS3DH_GetAccAxesRaw(&readData)) != MEMS_SUCCESS){
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

//uint16_t lis3dh_get_hp_filter_state(accelerometer_driver_t* accel_driver);
//float32_t lis3dh_get_hp_filter_cutoff_freq(accelerometer_driver_t* accel_driver);

