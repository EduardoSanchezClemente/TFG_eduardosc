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
 * output_packet.c
 *
 *  Created on: 6/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file output_packet.c
 */

#include "output_packet.h"

osMutexDef (output_packet_mutex);

/**
 * @brief	Creates a new output packet object
 * @return	The pointer to the created output packet
 */
output_packet_t* init_output_packet(void){
	output_packet_t* new_output_packet = (output_packet_t*) pvPortMalloc(sizeof(output_packet_t));

	new_output_packet->output_packet_mutex_id = osMutexCreate (osMutex (output_packet_mutex));

	new_output_packet->packet_header.bytes_per_data = 0;
	new_output_packet->packet_header.component_class = NO_CMP;
	new_output_packet->packet_header.data_length = 0;
	new_output_packet->packet_header.data_timestamp = 0;
	new_output_packet->packet_header.total_num_bytes = 0;

	return new_output_packet;
}

/**
 * @brief 					Removes a previously generated output packet object
 * @param 	output_packet	The output packet object to remove
 * @return					Return STATUS
 */
retval_t remove_output_packet(output_packet_t* output_packet){

	if(output_packet != NULL){
		osMutexDelete(output_packet->output_packet_mutex_id);
		vPortFree(output_packet);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief 					This function set the necessary atributes of a packet. With this function and if the correct payload has been set,
 * 							the packet is ready to be sent
 * @param output_packet		The output packet object
 * @param data_timestamp	Timestamp of the data. This timestamp should be the time of the first data byte. The timestamp can be obtained with get_timestamp(time_control_t* time_control);
 * @param data_len			Length of the data to be sent
 * @param bytes_per_data	Bytes of each datum information. For example: 4 if the data is of type float32_t
 * @param component_class	Information of the type of data to be sent, depending on the component (ACELEROMETER, RADAR, ETC)
 * @note					data_len*bytes_per_data must be lower than MAX_PAYLOAD_LEN
 * @return					Return STATUS
 */
retval_t set_packet_attr(output_packet_t* output_packet, uint64_t data_timestamp, uint16_t data_len, uint16_t bytes_per_data, component_class_t component_class){

	if(output_packet != NULL){
		if((data_len * bytes_per_data) > MAX_PAYLOAD_LEN){
			return RET_ERROR;
		}
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		output_packet->preamb = PREAMB;
		output_packet->packet_header.bytes_per_data = bytes_per_data;
		output_packet->packet_header.component_class = component_class;
		output_packet->packet_header.data_length = data_len;
		output_packet->packet_header.data_timestamp = data_timestamp;
		output_packet->packet_header.total_num_bytes = (data_len * bytes_per_data) + PREAMB_SIZE + sizeof(packet_header_t);
		osMutexRelease(output_packet->output_packet_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief				Sets the packet preamb
 * @param output_packet The output packet object
 * @param preamb		New preamb to be set
 * @return				Return STATUS
 */
retval_t set_packet_preamb(output_packet_t* output_packet, uint16_t preamb){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		output_packet->preamb = PREAMB;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief					Sets the packet timestamp
 * @param output_packet		The output packet object
 * @param data_timestamp	New timestamp to be set
 * @return					Return STATUS
 */
retval_t set_packet_timestamp(output_packet_t* output_packet, uint64_t data_timestamp){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		output_packet->packet_header.data_timestamp = data_timestamp;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief					Sets the packet data_length. Also sets the packet total number of bytes depending on the previously saved bytes_per_data
 * @param output_packet		The output packet object
 * @param data_length		New data length to be set
 * @return					Return STATUS
 */
retval_t set_packet_data_length(output_packet_t* output_packet, uint16_t data_length){
	if(output_packet != NULL){
		if((data_length * output_packet->packet_header.bytes_per_data) > MAX_PAYLOAD_LEN){
			return RET_ERROR;
		}
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		output_packet->packet_header.data_length = data_length;
		output_packet->packet_header.total_num_bytes = (data_length * output_packet->packet_header.bytes_per_data) + PREAMB_SIZE + sizeof(packet_header_t);
		osMutexRelease(output_packet->output_packet_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
/**
 * @brief					Sets the packet bytes per data. Also sets the packet total number of bytes depending on the previously saved data length
 * @param output_packet		The output packet object
 * @param bytes_per_data	New bytes per data to be set
 * @return					Return STATUS
 */
retval_t set_packet_bytes_per_data(output_packet_t* output_packet, uint16_t bytes_per_data){
	if(output_packet != NULL){
		if((output_packet->packet_header.data_length * bytes_per_data) > MAX_PAYLOAD_LEN){
			return RET_ERROR;
		}
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		output_packet->packet_header.bytes_per_data = bytes_per_data;
		output_packet->packet_header.total_num_bytes = (output_packet->packet_header.data_length * bytes_per_data) + PREAMB_SIZE + sizeof(packet_header_t);
		osMutexRelease(output_packet->output_packet_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
/**
 * @brief					Sets the packet component class
 * @param output_packet		The output packet object
 * @param component_class	New component class to be set
 * @return					Return STATUS
 */
retval_t set_packet_component_class(output_packet_t* output_packet, component_class_t component_class){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		output_packet->packet_header.component_class = component_class;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
/**
* @brief 					This function set the necessary atributes of a packet. This function also copies the data in payload parameter to
* 							the packet payload
* @param output_packet		The output packet object
* @param payload			The  data payload to be copied to the packet payload
* @param data_timestamp		Timestamp of the data. This timestamp should be the time of the first data byte. The timestamp can be obtained with get_timestamp(time_control_t* time_control);
* @param data_len			Length of the data to be sent
* @param bytes_per_data		Bytes of each datum information. For example: 4 if the data is of type float32_t
* @param component_class	Information of the type of data to be sent, depending on the component (ACELEROMETER, RADAR, ETC)
* @note						data_len*bytes_per_data must be lower than MAX_PAYLOAD_LEN
* @return					Return STATUS
*/
retval_t set_packet_full(output_packet_t* output_packet, uint8_t* payload, uint64_t data_timestamp, uint16_t data_len, uint16_t bytes_per_data, component_class_t component_class){
	if(output_packet != NULL){
		if((data_len * bytes_per_data) > MAX_PAYLOAD_LEN){
			return RET_ERROR;
		}
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		output_packet->preamb = PREAMB;
		output_packet->packet_header.bytes_per_data = bytes_per_data;
		output_packet->packet_header.component_class = component_class;
		output_packet->packet_header.data_length = data_len;
		output_packet->packet_header.data_timestamp = data_timestamp;
		output_packet->packet_header.total_num_bytes = (data_len * bytes_per_data) + PREAMB_SIZE + sizeof(packet_header_t);
		memcpy(output_packet->payload, payload, (data_len * bytes_per_data));
		osMutexRelease(output_packet->output_packet_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

/**
 * @brief				Returns the current preamb of the output packet object
 * @param output_packet The output packet object
 * @return				Current Preamb. 0 if error
 */
uint16_t get_packet_preamb(output_packet_t* output_packet){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		uint16_t preamb = output_packet->preamb;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return preamb;
	}
	else{
		return 0;
	}
}

/**
 * @brief				Returns the current timestamp of the output packet object
 * @param output_packet The output packet object
 * @return				Current output packet timestamp. 0 if error
 */
uint64_t get_packet_timestamp(output_packet_t* output_packet){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		uint64_t timestamp = output_packet->packet_header.data_timestamp;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return timestamp;
	}
	else{
		return 0;
	}
}

/**
 * @brief				Returns the current total number of bytes of the output packet object
 * @param output_packet	The output packet object
 * @return				Current total number of bytes. 0 if error
 */
uint16_t get_packet_total_num_bytes(output_packet_t* output_packet){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		uint16_t total_num_bytes = output_packet->packet_header.total_num_bytes;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return total_num_bytes;
	}
	else{
		return 0;
	}
}

/**
 * @brief				Returns the current data length of the output packet object
 * @param output_packet	The output packet object
 * @return				Current data length. 0 if error
 */
uint16_t get_packet_data_length(output_packet_t* output_packet){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		uint16_t data_length = output_packet->packet_header.data_length;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return data_length;
	}
	else{
		return 0;
	}
}

/**
 * @brief				Returns the current bytes per data of the output packet object
 * @param output_packet	The output packet object
 * @return				Current Bytes per data. 0 if error
 */
uint16_t get_packet_bytes_per_data(output_packet_t* output_packet){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		uint16_t bytes_per_data = output_packet->packet_header.bytes_per_data;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return bytes_per_data;
	}
	else{
		return 0;
	}
}

/**
 * @brief				Returns the current component class of the output packet object
 * @param output_packet The output packet object
 * @return				Current Component Class. NO_CMP if error
 */
component_class_t get_packet_component_class(output_packet_t* output_packet){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		component_class_t component_class = output_packet->packet_header.component_class;
		osMutexRelease(output_packet->output_packet_mutex_id);
		return component_class;
	}
	else{
		return NO_CMP;
	}
}


/**
 * @brief				Catch and returns the output packet payload pointer, so it can be filled with data. After the data is written or read
 * 						into the payload pointer, it MUST be released with the release_packet_payload_ptr() function before using any other output_packet function
 * @param output_packet The output packet object
 * @return				A pointer to the output packet payload. NULL if error
 */
uint8_t* catch_packet_payload_ptr(output_packet_t* output_packet){
	if(output_packet != NULL){
		osMutexWait(output_packet->output_packet_mutex_id, osWaitForever);
		uint8_t* payload_ptr = output_packet->payload;
		return payload_ptr;
	}
	else{
		return NULL;
	}
}

/**
 * @brief				Releases the lock on the packet payload pointer after it has been captured by catch_packet_payload_ptr() function.
 * 						No writing or reading can be done in payload parameter after this function.
 * @param output_packet	The output packet object
 * @param payload		The payload pointer to be released
 * @return				Return STATUS
 */
retval_t  release_packet_payload_ptr(output_packet_t* output_packet, uint8_t* payload){
	if(output_packet != NULL){
		if(payload != NULL){
			payload = NULL;
			osMutexRelease(output_packet->output_packet_mutex_id);
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
