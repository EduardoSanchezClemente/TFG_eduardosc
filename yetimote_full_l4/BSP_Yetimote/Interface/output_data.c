/*
 * Copyright (c) 2016, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * output_data.c
 *
 *  Created on: 6/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file output_data.c
 */


#include "output_data.h"

osMutexDef (output_data_mutex);

/* **************************************************************/
/* *******Output interfaces function declaration ****************/
retval_t usb_output(struct output_data_* output_data, uint8_t* data, uint16_t size);
/* **************************************************************/

/**
 * @brief 			Create a new output data object. send done_func is NULL by default. Data ready is 0 and active is 0
 * @param interface Default interface used by the output
 * @return			The new object pointer. If error returns NULL
 */
output_data_t* init_output_data(output_interface_t interface){

	output_data_t* new_output_data = (output_data_t*) pvPortMalloc(sizeof(output_data_t));

	new_output_data->output_flags= 0;		//Is active and data ready set to 0 by default

	switch(interface){
	case OUT_USB:
		new_output_data->interface = OUT_USB;
		new_output_data->output_data_func = usb_output;
		new_output_data->send_done_func = NULL;		//By default there is no send_done_function. It must be done by the user
		break;
	default:
		vPortFree(new_output_data);
		return NULL;
		break;
	}

	new_output_data->output_data_mutex_id = osMutexCreate (osMutex (output_data_mutex));

	return new_output_data;
}

/**
 * @brief				Removes a previously generated output data object
 * @param output_data 	The output data object to remove
 * @return				Return STATUS
 */
retval_t remove_output_data(output_data_t* output_data){

	if(output_data != NULL){
		osMutexDelete(output_data->output_data_mutex_id);
		vPortFree(output_data);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}

}

/**
 * @brief				Gets the output state. Active or disabled
 * @param output_data	The output data object
 * @return				Output active state. 0 if output is disabled
 */
uint8_t is_output_active(output_data_t* output_data){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		uint8_t is_active = output_data->output_flags & OUTPUT_ACTIVE_BIT_MASK;
		osMutexRelease(output_data->output_data_mutex_id);
		return is_active;
	}
	else{
		return 0;
	}
}

/**
 * @brief				Activates the output
 * @param output_data	The output data object
 * @return				Return STATE
 */
retval_t output_active(output_data_t* output_data){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		output_data->output_flags |= OUTPUT_ACTIVE_BIT_MASK;
		osMutexRelease(output_data->output_data_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief				Disables the output
 * @param output_data	The output data object
 * @return				Return STATE
 */
retval_t output_disable(output_data_t* output_data){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		output_data->output_flags &= ~OUTPUT_ACTIVE_BIT_MASK;
		osMutexRelease(output_data->output_data_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief				Gets the output data ready state.
 * @param output_data	The output data object
 * @return				Returns 1 if data is ready, 0 if not
 */
uint8_t is_output_data_ready(output_data_t* output_data){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		uint8_t is_data_ready = (output_data->output_flags & DATA_READY_BIT_MASK) >> 1;
		osMutexRelease(output_data->output_data_mutex_id);
		return is_data_ready;
	}
	else{
		return 0;
	}
}

/**
 * @brief				Sets data ready bit
 * @param output_data	The output data object
 * @return				Return STATE
 */
retval_t output_set_data_ready(output_data_t* output_data){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		output_data->output_flags |= DATA_READY_BIT_MASK;
		osMutexRelease(output_data->output_data_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief				Resets data ready bit
 * @param output_data	The output data object
 * @return				return STATE
 */
retval_t output_reset_data_ready(output_data_t* output_data){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		output_data->output_flags &= ~DATA_READY_BIT_MASK;
		osMutexRelease(output_data->output_data_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief				Returns the current used interface by the output
 * @param output_data	The output data object
 * @return
 */
output_interface_t output_get_interface(output_data_t* output_data){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		output_interface_t output_interface = output_data->interface;
		osMutexRelease(output_data->output_data_mutex_id);
		return output_interface;
	}
	else{
		return 0;
	}
}

/**
 * @brief 				Sets the interface used to output the data
 * @param output_data	The output data object
 * @param interface		New output interface for the output data
 * @return				Return STATE
 */
retval_t output_set_interface(output_data_t* output_data, output_interface_t interface){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		switch(interface){
			case OUT_USB:
				output_data->interface = OUT_USB;
				output_data->output_data_func = usb_output;
				break;
			default:
				return RET_ERROR;
				break;
			}
		output_data->interface = interface;
		osMutexRelease(output_data->output_data_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief					Sets the the function called after a data transmision
 * @param output_data		The output data object
 * @param send_done_func	New send done function. If NULL no function will be called
 * @return					Returns STATE
 */
retval_t output_set_send_done_function(output_data_t* output_data, retval_t (* send_done_func)(struct output_data_* output_data)){
	if(output_data != NULL){
		osMutexWait(output_data->output_data_mutex_id, osWaitForever);
		output_data->send_done_func = send_done_func;
		osMutexRelease(output_data->output_data_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/* **************************************************************/
/* *******Output interfaces functions  **************************/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/**
 * @brief				Outputs a buffer of data using USB. Blocks until done and then calls the send done function.
 * @param output_data	The output data object
 * @param data			Data buffer to be sent
 * @param size			Size of the data to send
 * @return				Return STATE
 */
retval_t usb_output(struct output_data_* output_data, uint8_t* data, uint16_t size){
	if(output_data != NULL){

		if(is_output_active(output_data)){
			PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
			USBD_HandleTypeDef  *pdev = hpcd->pData;
			USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;

			if(pdev->dev_state != USBD_STATE_SUSPENDED){	//De este modo nos aseguramos que no se envia nada si el cable USB no está conectado
				CDC_Transmit_FS(data, size);				//Si no entraria en infinite loop al usar un printf
				while(hcdc->TxState == 1);					//Waits until the transmision is done
				if(output_data->send_done_func != NULL){
					output_data->send_done_func(output_data);
				}

			}
			else{
				return RET_ERROR;
			}
			return RET_OK;
		}
		else{
			return RET_ERROR;
		}
	}
	else{
		return RET_ERROR;
	}
}
/* **************************************************************/
