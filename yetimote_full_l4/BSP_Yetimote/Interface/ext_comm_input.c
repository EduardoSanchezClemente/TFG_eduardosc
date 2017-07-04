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
 * ext_comm_input.c
 *
 *  Created on: 1/6/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file ext_comm_input.c
 */

#include "ext_comm_input.h"

osMutexDef (circular_buffer_mutex);

retval_t check_add_char(input_data_t* input, uint8_t new_char, int32_t* prev_new_char_number, uint16_t* command_index, uint16_t* echo_index);

input_data_t* new_input_data(uint16_t buffer_size){

	input_data_t* new_input_data = (input_data_t*) pvPortMalloc(sizeof(input_data_t));
	new_input_data->input_commands_str = (uint8_t*) pvPortMalloc(buffer_size*sizeof(uint8_t)+1);		//Reserves an extra space for managing the '\0' character in the worst case
	new_input_data->circular_buffer = (uint8_t*) pvPortMalloc(CIRCULAR_BUFFER_SIZE*sizeof(uint8_t));
	new_input_data->rcv_usb_buffer_counter = CIRCULAR_BUFFER_SIZE;
	new_input_data->head = new_input_data->circular_buffer;
	new_input_data->tail = new_input_data->circular_buffer;
	new_input_data->input_command_str_count = 0;
	new_input_data->input_command_str_current_count = 0;
	new_input_data->max_cmd_buffer_size = buffer_size;
	new_input_data->newline_flag = NO_NEWLINE;
	new_input_data->circular_buffer_mutex = osMutexCreate(osMutex(circular_buffer_mutex));

	new_input_data->command = new_command_data();
	new_input_data->echo = ECHO_ON;
	new_input_data->echo_output = (uint8_t*) pvPortMalloc(ECHO_BUFFER_SIZE*sizeof(uint8_t)+1);

	return new_input_data;
}
retval_t remove_input_data(input_data_t* input_data){

	if(input_data != NULL){
		vPortFree(input_data->input_commands_str);
		vPortFree(input_data->circular_buffer);
		vPortFree(input_data->echo_output);
		remove_command_data(input_data->command);
		vPortFree(input_data);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

retval_t start_listening(input_data_t* input_data){
	if(input_data != NULL){

//		HAL_UART_Receive_DMA(&huart2, input_data->circular_buffer, CIRCULAR_BUFFER_SIZE);

		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

retval_t stop_listening(input_data_t* input_data){
	if(input_data != NULL){

//		HAL_UART_DMAStop(&huart2);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

retval_t pause_listening(input_data_t* input_data){
	if(input_data != NULL){

//		HAL_UART_DMAPause(&huart2);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

retval_t resume_listening(input_data_t* input_data){
	if(input_data != NULL){

//		HAL_UART_DMAResume(&huart2);

		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}


retval_t get_new_data(input_data_t* input_data){

	if(input_data != NULL){
		osMutexWait (input_data->circular_buffer_mutex, SYSTEM_CONF_SECOND/10);	//Protect access to input_data->circular_buffer

		input_data->head = input_data->circular_buffer + CIRCULAR_BUFFER_SIZE - input_data->rcv_usb_buffer_counter;

		if(input_data->head > input_data->tail){
			uint16_t i;
			uint16_t command_index = 0;
			uint16_t echo_index = 0;
			int32_t new_chars_number = input_data->head - input_data->tail;


			if((input_data->input_command_str_count + new_chars_number) > input_data->max_cmd_buffer_size){
				input_data->input_command_str_count = 0;
				input_data->input_command_str_current_count = 0;
				return RET_ERROR;	//COMMAND BUFFER OVERFLOW
			}

			for(i=0; i<new_chars_number; i++){

				check_add_char(input_data, input_data->tail[i], &new_chars_number, &command_index, & echo_index);

				command_index ++;
				if(input_data->echo == ECHO_ON){
					echo_index ++;
				}
			}
			if(input_data->echo == ECHO_ON){
				input_data->echo_output[echo_index] = '\0';
				UsbWriteString((char*)input_data->echo_output);
			}

			input_data->input_command_str_count = input_data->input_command_str_count + new_chars_number;
			input_data->tail = input_data->head;
		}

		else if(input_data->head < input_data->tail){

			uint16_t i;
			uint16_t command_index = 0;
			uint16_t echo_index = 0;
			uint32_t diff_end = input_data->circular_buffer + CIRCULAR_BUFFER_SIZE - input_data->tail;
			uint32_t diff_start = input_data->head - input_data->circular_buffer;
			int32_t new_chars_number = (int32_t) diff_end + diff_start;

			if((input_data->input_command_str_count + new_chars_number) > input_data->max_cmd_buffer_size){
				input_data->input_command_str_count = 0;
				input_data->input_command_str_current_count = 0;
				return RET_ERROR;	//COMMAND BUFFER OVERFLOW
			}

			for(i=0; i<new_chars_number; i++){
				if( i < diff_end){
					check_add_char(input_data, input_data->tail[i], &new_chars_number, &command_index, &echo_index);
				}
				else{
					check_add_char(input_data, input_data->circular_buffer[i-diff_end], &new_chars_number, &command_index, &echo_index);
				}
				command_index ++;
				if(input_data->echo == ECHO_ON){
					echo_index ++;
				}
			}

			if(input_data->echo == ECHO_ON){
				input_data->echo_output[echo_index] = '\0';
				UsbWriteString((char*)input_data->echo_output);
			}

			input_data->input_command_str_count = input_data->input_command_str_count + new_chars_number;
			input_data->tail = input_data->head;

		}
		osMutexRelease (input_data->circular_buffer_mutex);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}


retval_t check_newline(input_data_t* input_data){

	if(input_data != NULL){
		osMutexWait (input_data->circular_buffer_mutex, SYSTEM_CONF_SECOND/10);	//Protect access to input_data->circular_buffer
		uint16_t i;
		uint16_t j;

		for(i=input_data->input_command_str_current_count; i <input_data->input_command_str_count; i++){		//Now check for a newline. The new line could be '\r' '\n' or '\r\n'

			if((input_data->input_commands_str[i] == '\r') || (input_data->input_commands_str[i] == '\n')){

				input_data->input_commands_str[i] = '\0';

				input_data->command->argv = str_split((char*) input_data->input_commands_str, ' ', (int*) &input_data->command->argc );	//Memory for argv is reserved inside str_split function. It is necessary to free it later

				uint16_t extra_bytes = input_data->input_command_str_count-i-1;		//Number of existing bytes after '\r' or '\n'

				uint16_t rpt_new_line = 0;
				if(extra_bytes > 0){
					if((input_data->input_commands_str[i+1] == '\r') || (input_data->input_commands_str[i+1] == '\n')){
						extra_bytes--;
						rpt_new_line = 1;
					}
				}
				for(j=0; j<extra_bytes; j++){				//Moves everything (if exists) after '\r' or '\n' to the start of the buffer

					input_data->input_commands_str[j] = input_data->input_commands_str[i+j+1+rpt_new_line];

				}

				input_data->input_command_str_current_count = 0;
				input_data->input_command_str_count = extra_bytes;

				osMutexRelease (input_data->circular_buffer_mutex);
				return RET_OK;

			}
		}

		input_data->input_command_str_current_count = input_data->input_command_str_count;

		osMutexRelease (input_data->circular_buffer_mutex);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

retval_t check_add_char(input_data_t* input_data, uint8_t new_char, int32_t* prev_new_char_number, uint16_t* command_index, uint16_t* echo_index){
	if(input_data != NULL){

		switch (new_char){
		case '\b':				//Borrar caracter

			if(input_data->input_command_str_current_count > 0){			//Reduzco una posicion del buffer guardado
				input_data->input_command_str_current_count--;
			}

			if(input_data->input_command_str_count > 0){					//Se resta dos al numero de caracteres nuevos (el anterior a borrar y el propio \b)
				(*prev_new_char_number) = (*prev_new_char_number)- 2;
			}
			else{
				(*prev_new_char_number)--;									//Solo se resta el comando \b (no hay anteriores)
			}

			if((*command_index) > 0){										//Se resta dos al indice (el anterior a borrar y el propio \b)
				(*command_index) = (*command_index)- 2;
			}
			else{
				(*command_index)--;											//Solo se resta el indice de \b (no hay anteriores)
			}




			if(input_data->echo == ECHO_ON){
				if(input_data->input_command_str_count > 0){
					if((*echo_index) < ECHO_BUFFER_SIZE-2){
						input_data->echo_output[(*echo_index)] = '\b';
						input_data->echo_output[++(*echo_index)] = ' ';
						input_data->echo_output[++(*echo_index)] = '\b';
					}
				}
				else{
						(*echo_index)--;
				}
			}

			break;
		case '\r':				//En caso de recibir \r imprimir por echo \r\n
			if(((*command_index)+input_data->input_command_str_count) < input_data->max_cmd_buffer_size){
				input_data->input_commands_str[(*command_index)+input_data->input_command_str_count] = new_char;
			}

			if(input_data->echo == ECHO_ON){
				if((*echo_index) < ECHO_BUFFER_SIZE-1){
					input_data->echo_output[(*echo_index)] = '\r';
					input_data->echo_output[++(*echo_index)] = '\n';
				}
			}
			break;
		case '\n':				//En caso de recibir \n no imprimir nada por echo
			if(((*command_index)+input_data->input_command_str_count) < input_data->max_cmd_buffer_size){
				input_data->input_commands_str[(*command_index)+input_data->input_command_str_count] = new_char;
			}
			if(input_data->echo == ECHO_ON){
				(*echo_index)--;
			}
			break;
		default:
			if(((*command_index)+input_data->input_command_str_count) < input_data->max_cmd_buffer_size){
				input_data->input_commands_str[(*command_index)+input_data->input_command_str_count] = new_char;
			}
			if(input_data->echo == ECHO_ON){
				input_data->echo_output[(*echo_index)] = new_char;
			}
			break;
		}

		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
