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
 * output_packet.h
 *
 *  Created on: 3/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file output_packet.h
 */

#ifndef APPLICATION_USER_BSP_YETIMOTE_INTERFACE_OUTPUT_PACKET_H_
#define APPLICATION_USER_BSP_YETIMOTE_INTERFACE_OUTPUT_PACKET_H_

#include "yetimote-conf.h"
#include <string.h>

#define MAX_PAYLOAD_LEN 1024

#define PREAMB_SIZE 	sizeof(uint16_t)
#define PREAMB			0xAA

typedef enum component_class{
	ACCELEROMETER = 1,
	RADAR = 2,
	TEMP = 3,
	DEBUG = 4,
	WARNING = 5,
	MICROPHONE = 6,
	STRING = 7,
	NO_CMP = 65535,		//This way component_class_t has a size of 16 bits
}component_class_t;

typedef struct __packed packet_header_{
	uint64_t data_timestamp; 				//Timestamp del dato del mensaje

	uint16_t total_num_bytes;				//Numero total de bytes enviados incluyendo header y preamb
	uint16_t data_length;					//Numero de datos efectivos que se transmiten
	uint8_t bytes_per_data;				//Numero de bytes de cada dato

	component_class_t component_class;		//Tipo de datos especificos que se van a transmitir

}packet_header_t;

typedef struct __packed output_packet_{
	uint8_t preamb;		//2 Bytes de preambulo
	packet_header_t packet_header;
	uint8_t payload[MAX_PAYLOAD_LEN];

	osMutexId output_packet_mutex_id;
}output_packet_t;

output_packet_t* init_output_packet(void);
retval_t remove_output_packet(output_packet_t* output_packet);

retval_t set_packet_attr(output_packet_t* output_packet, uint64_t data_timestamp, uint16_t data_len, uint16_t bytes_per_data, component_class_t component_class);

retval_t set_packet_preamb(output_packet_t* output_packet, uint16_t preamb);
retval_t set_packet_timestamp(output_packet_t* output_packet, uint64_t data_timestamp);
retval_t set_packet_data_length(output_packet_t* output_packet, uint16_t data_length);
retval_t set_packet_bytes_per_data(output_packet_t* output_packet, uint16_t bytes_per_data);
retval_t set_packet_component_class(output_packet_t* output_packet, component_class_t component_class);
retval_t set_packet_full(output_packet_t* output_packet, uint8_t* payload, uint64_t data_timestamp, uint16_t data_len, uint16_t bytes_per_data, component_class_t component_class);

uint16_t get_packet_preamb(output_packet_t* output_packet);
uint64_t get_packet_timestamp(output_packet_t* output_packet);
uint16_t get_packet_total_num_bytes(output_packet_t* output_packet);
uint16_t get_packet_data_length(output_packet_t* output_packet);
uint16_t get_packet_bytes_per_data(output_packet_t* output_packet);
component_class_t get_packet_component_class(output_packet_t* output_packet);

uint8_t* catch_packet_payload_ptr(output_packet_t* output_packet);		//This function has a mutex that locks until packet payload has been released.
retval_t  release_packet_payload_ptr(output_packet_t* output_packet, uint8_t* payload);

#endif /* APPLICATION_USER_BSP_YETIMOTE_INTERFACE_OUTPUT_PACKET_H_ */
