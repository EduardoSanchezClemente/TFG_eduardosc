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
 * output_data.h
 *
 *  Created on: 16/9/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file output_data.h
 */

#ifndef APPLICATION_USER_BSP_YETIMOTE_INTERFACE_OUTPUT_DATA_H_
#define APPLICATION_USER_BSP_YETIMOTE_INTERFACE_OUTPUT_DATA_H_

#include "yetimote-conf.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define OUTPUT_ACTIVE_BIT_MASK	0x01
#define DATA_READY_BIT_MASK		0x02

typedef enum output_interface_{
	OUT_USB,
	OUT_UART,
	OUT_RADIO,
	OUT_WIFI,
	OUT_NONE,
}output_interface_t;

typedef struct output_data_{
	uint8_t output_flags;											//1st bit: Output active bit. 2nd bit: Data ready bit
	output_interface_t interface;
	retval_t (* output_data_func)(struct output_data_*, uint8_t* data, uint16_t size);
	retval_t (* send_done_func)(struct output_data_*);

	osMutexId output_data_mutex_id;
}output_data_t;


output_data_t* init_output_data(output_interface_t interface);
retval_t remove_output_data(output_data_t* output_data);

uint8_t is_output_active(output_data_t* output_data);
retval_t output_active(output_data_t* output_data);
retval_t output_disable(output_data_t* output_data);

uint8_t is_output_data_ready(output_data_t* output_data);
retval_t output_set_data_ready(output_data_t* output_data);
retval_t output_reset_data_ready(output_data_t* output_data);

output_interface_t output_get_interface(output_data_t* output_data);
retval_t output_set_interface(output_data_t* output_data, output_interface_t interface);

retval_t output_set_send_done_function(output_data_t* output_data, retval_t (* send_done_func)(output_data_t* output_data));

#endif /* APPLICATION_USER_BSP_YETIMOTE_INTERFACE_OUTPUT_DATA_H_ */
