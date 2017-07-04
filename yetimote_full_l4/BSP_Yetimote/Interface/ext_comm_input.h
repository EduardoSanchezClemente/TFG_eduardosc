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
 * ext_comm_input.h
 *
 *  Created on: 1/6/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file ext_comm_input.h
 */

#ifndef APPLICATION_USER_S4BIM_DSP_CODE_COMMUNICATIONS_EXT_COMM_INPUT_H_
#define APPLICATION_USER_S4BIM_DSP_CODE_COMMUNICATIONS_EXT_COMM_INPUT_H_

#include "yetimote-conf.h"
#include "leds.h"
#include "commands.h"
#include "strfunc.h"

#define MAX_CMD_INPUT_BUFFER	256

#define	CIRCULAR_BUFFER_SIZE	256

#define	ECHO_BUFFER_SIZE		256

enum{
	NO_NEWLINE,
	NEWLINE
};

enum{
	ECHO_OFF,
	ECHO_ON
};


typedef struct input_data{
	uint8_t* circular_buffer;
	uint16_t rcv_usb_buffer_counter;
	uint8_t* input_commands_str;
	uint8_t* head;
	uint8_t* tail;
	uint16_t input_command_str_count;
	uint16_t input_command_str_current_count;
	uint16_t max_cmd_buffer_size;
	command_data_t* command;
	uint8_t newline_flag;
	uint8_t echo;
	uint8_t* echo_output;
	osMutexId circular_buffer_mutex;
}input_data_t;

input_data_t* new_input_data(uint16_t buffer_size);
retval_t remove_input_data(input_data_t* input_data);

retval_t start_listening(input_data_t* input_data);
retval_t stop_listening(input_data_t* input_data);
retval_t pause_listening(input_data_t* input_data);
retval_t resume_listening(input_data_t* input_data);

retval_t get_new_data(input_data_t* input_data);
retval_t check_newline(input_data_t* input_data);

#endif /* APPLICATION_USER_S4BIM_DSP_CODE_COMMUNICATIONS_EXT_COMM_INPUT_H_ */
