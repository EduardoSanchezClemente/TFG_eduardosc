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
 * config_commands.c
 *
 *  Created on: 7/6/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file config_commands.c
 */


#include "commands.h"

extern time_control_t* time_control;
extern broadcast_comm_t* bc_comm;
extern unicast_comm_t* uc_comm;
extern mesh_comm_t* msh_comm;

retval_t null_func(uint16_t argc, char** argv);

retval_t no_cmd_command(uint16_t argc, char** argv);
retval_t error_wrong_params(uint16_t argc, char** argv);
retval_t error_no_param_command(uint16_t argc, char** argv);

retval_t list_cmd_command(uint16_t argc, char** argv);
retval_t print_mem_command(uint16_t argc, char** argv);
retval_t print_addr_command(uint16_t argc, char** argv);
retval_t open_comm_channel_command(uint16_t argc, char** argv);
retval_t send_message_on_channel(uint16_t argc, char** argv);
retval_t change_send_delay(uint16_t argc, char** argv);
retval_t change_send_packet_num(uint16_t argc, char** argv);
retval_t send_test_unicast_stream(uint16_t argc, char** argv);
retval_t send_test_mesh_stream(uint16_t argc, char** argv);
retval_t change_radio(uint16_t argc, char** argv);
retval_t change_packet_length(uint16_t argc, char** argv);
retval_t get_time(uint16_t argc, char** argv);
retval_t set_time(uint16_t argc, char** argv);
//extern retval_t send_config(uint16_t argc, char** argv);
extern retval_t acc_config(uint16_t argc, char** argv);

//Radio comm callbacks//
extern void broadcast_print(struct broadcast_conn *c, const linkaddr_t *from);
extern void broadcast_noprint(struct broadcast_conn *c, const linkaddr_t *from);
extern void unicast_print(struct unicast_conn *c, const linkaddr_t *from);
extern void unicast_noprint(struct unicast_conn *c, const linkaddr_t *from);
extern void mesh_print(struct mesh_conn *c, const linkaddr_t *from, uint8_t hops);
extern void mesh_noprint(struct mesh_conn *c, const linkaddr_t *from, uint8_t hops);
/*////////////////////*/

command_data_t* new_command_data(){

	command_data_t* new_command_data = (command_data_t*) pvPortMalloc(sizeof(command_data_t));
	new_command_data->argc = 0;
	new_command_data->command_func = null_func;

	return new_command_data;
}
retval_t remove_command_data(command_data_t* command_data){

	if(command_data != NULL){
		vPortFree(command_data);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}

}

retval_t select_command(command_data_t* command_data){
	if(command_data != NULL){
		if(command_data->argc != 0){
			/////////////////////////////////////////////////////////////////////
			if(strcmp(command_data->argv[0],LIST_CMD) == 0){
				command_data->command_func = list_cmd_command;
			}
			else if(strcmp(command_data->argv[0],PRINT_MEM) == 0){
				command_data->command_func = print_mem_command;
			}
			else if(strcmp(command_data->argv[0],PRINT_ADDR) == 0){
				command_data->command_func = print_addr_command;
			}
			else if(strcmp(command_data->argv[0],OPEN_COMM_CHANNEL) == 0){
				command_data->command_func = open_comm_channel_command;
			}
			else if(strcmp(command_data->argv[0],SEND_MESSAGE_ON_CHANNEL) == 0){
				command_data->command_func = send_message_on_channel;
			}
			else if(strcmp(command_data->argv[0],CHANGE_SEND_DELAY) == 0){
				command_data->command_func = change_send_delay;
			}
			else if(strcmp(command_data->argv[0],CHANGE_SEND_PACKET_NUM) == 0){
				command_data->command_func = change_send_packet_num;
			}
			else if(strcmp(command_data->argv[0],SEND_TEST_UNICAST_STREAM) == 0){
				command_data->command_func = send_test_unicast_stream;
			}
			else if(strcmp(command_data->argv[0],SEND_TEST_MESH_STREAM) == 0){
				command_data->command_func = send_test_mesh_stream;
			}
			else if(strcmp(command_data->argv[0],CHANGE_RADIO) == 0){
				command_data->command_func = change_radio;
			}
			else if(strcmp(command_data->argv[0],CHANGE_PACKET_LENGTH) == 0){
				command_data->command_func = change_packet_length;
			}
			else if(strcmp(command_data->argv[0],SET_TIME) == 0){
				command_data->command_func = set_time;
			}
			else if(strcmp(command_data->argv[0],GET_TIME) == 0){
				command_data->command_func = get_time;
			}
//			else if(strcmp(command_data->argv[0],CONFIG) == 0){
//				command_data->command_func = send_config;
//			}
			else if(strcmp(command_data->argv[0],ACC) == 0){
				command_data->command_func = acc_config;
			}
			else{
				command_data->command_func = no_cmd_command;
				return RET_ERROR;
			}
			return RET_OK;
		}
		else{
			return RET_OK;
		}

		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

retval_t execute_command(command_data_t* command_data){
	if(command_data != NULL){
		if(command_data->argc != 0){
			uint16_t i = 0;

			command_data->command_func(command_data->argc, command_data->argv);

			command_data->command_func = null_func;

			for (i=0; i<command_data->argc; i++){			//When a command is executed, the memory occupied by the args must be freed
				if(command_data->argv[i] != NULL){
					vPortFree(command_data->argv[i]);
					command_data->argv[i] = NULL;
				}
			}
			if(command_data->argv != NULL){
				vPortFree(command_data->argv);
				command_data->argv = NULL;
			}

			command_data->argc = 0;

			UsbWriteString(">");
		}

		return RET_OK;

	}
	else{
		return RET_ERROR;
	}
}


retval_t null_func(uint16_t argc, char** argv){
	return RET_OK;
}




/**
 * @brief	Default message shown when entered an invalid command
 */
retval_t no_cmd_command(uint16_t argc, char** argv){
	UsbWriteString(">Invalid command\r\n");
	return RET_OK;
}

/**
 * @brief	Function that showsa list of the existing available commands
 */
retval_t list_cmd_command(uint16_t argc, char** argv){

	UsbWriteString("Available commands:\r\n");

//////////////START COMMAND ZONE//////////////////////
#ifdef LIST_CMD
	UsbWriteString("-");
	UsbWriteString(LIST_CMD);
	UsbWriteString("\r\n");
#endif

#ifdef PRINT_MEM
	UsbWriteString("-");
	UsbWriteString(PRINT_MEM);
	UsbWriteString("\r\n");
#endif

#ifdef PRINT_ADDR
	UsbWriteString("-");
	UsbWriteString(PRINT_ADDR);
	UsbWriteString("\r\n");
#endif

#ifdef OPEN_COMM_CHANNEL
	UsbWriteString("-");
	UsbWriteString(OPEN_COMM_CHANNEL);
	UsbWriteString("\r\n");
#endif
#ifdef SEND_MESSAGE_ON_CHANNEL
	UsbWriteString("-");
	UsbWriteString(SEND_MESSAGE_ON_CHANNEL);
	UsbWriteString("\r\n");
#endif
#ifdef CHANGE_SEND_DELAY
	UsbWriteString("-");
	UsbWriteString(CHANGE_SEND_DELAY);
	UsbWriteString("\r\n");
#endif
#ifdef CHANGE_SEND_PACKET_NUM
	UsbWriteString("-");
	UsbWriteString(CHANGE_SEND_PACKET_NUM);
	UsbWriteString("\r\n");
#endif
#ifdef SEND_TEST_UNICAST_STREAM
	UsbWriteString("-");
	UsbWriteString(SEND_TEST_UNICAST_STREAM);
	UsbWriteString("\r\n");
#endif
#ifdef SEND_TEST_MESH_STREAM
	UsbWriteString("-");
	UsbWriteString(SEND_TEST_MESH_STREAM);
	UsbWriteString("\r\n");
#endif
#ifdef CHANGE_RADIO
	UsbWriteString("-");
	UsbWriteString(CHANGE_RADIO);
	UsbWriteString("\r\n");
#endif
#ifdef CHANGE_PACKET_LENGTH
	UsbWriteString("-");
	UsbWriteString(CHANGE_PACKET_LENGTH);
	UsbWriteString("\r\n");
#endif
#ifdef SET_TIME
	UsbWriteString("-");
	UsbWriteString(SET_TIME);
	UsbWriteString("\r\n");
#endif
#ifdef GET_TIME
	UsbWriteString("-");
	UsbWriteString(GET_TIME);
	UsbWriteString("\r\n");
#endif
//////////////END COMMAND ZONE//////////////////////
	return RET_OK;
}


/**
 * @brief	Shows the available memory on FreeRTOS in KB
 */
retval_t print_mem_command(uint16_t argc, char** argv){
//	DataToConvertType data;
	char* sprintfString = pvPortMalloc(MAX_OUT_STR_BUFF);
	size_t freeMEM = xPortGetFreeHeapSize();
	sprintf(sprintfString, "%.3f", (float) ((float)freeMEM/1000));

//
//
//	data.dataType = FLOAT;
//	data.data = &freeMemKbytes;
//	dataToString(data, sprintfString);

	UsbWriteString(">Available Memory: ");
	UsbWriteString(sprintfString);
	UsbWriteString(" KB\r\n");

	vPortFree(sprintfString);					//El calculo hecho por la hebra paralela con sprintf se almacena en esta variable, cuya memoria hay que liberar una vez usada
	return RET_OK;
}

/**
 * @brief	Shows the rime address of the node
 */
retval_t print_addr_command(uint16_t argc, char** argv){
//	DataToConvertType data;
	uint32_t addrByte;
	char* sprintfString = pvPortMalloc(MAX_OUT_STR_BUFF);

//	data.dataType = UNSIGNED_DEC_INT32;
	addrByte = linkaddr_node_addr.u8[0];
//	data.data = &addrByte;
//	dataToString(data, sprintfString);

	sprintf(sprintfString, "%d",(int) addrByte);
	UsbWriteString("Node address (Bytes): ");
	UsbWriteString(sprintfString);
	UsbWriteString(".");

//	data.dataType = UNSIGNED_DEC_INT32;
	addrByte = linkaddr_node_addr.u8[1];
//	data.data = &addrByte;
//	dataToString(data, sprintfString);
	sprintf(sprintfString, "%d",(int) addrByte);
	UsbWriteString(sprintfString);
	UsbWriteString("\r\n");

//	data.dataType = UNSIGNED_DEC_INT32;
	addrByte = linkaddr_node_addr.u16;
//	data.data = &addrByte;
//	dataToString(data, sprintfString);
	sprintf(sprintfString, "%d",(int) addrByte);
	UsbWriteString("Node address (Dec): ");
	UsbWriteString(sprintfString);
	UsbWriteString("\r\n");

	vPortFree(sprintfString);
	return RET_OK;
}

/**
 * @brief	Prints an error when there are some missing params for command execution
 */
retval_t error_no_param_command(uint16_t argc, char** argv){

	UsbWriteString("Command error. Missing params\r\n");
	return RET_OK;
}

/**
 * @brief	Prints an error when there are too much params or wrong params
 */
retval_t error_wrong_params(uint16_t argc, char** argv){
	UsbWriteString("Command error. Wrong params\r\n");
	return RET_OK;
}


/**
 * @brief 			Open a radio communication channel
 * @param argc		Number of args passed to the command
 * @param argv		Pointer to args passed to the command
 */
retval_t open_comm_channel_command(uint16_t argc, char** argv){

	///Solo se ha escrito el nombre del comando sin argumentos. Da error
	if(argc == 1){
		error_no_param_command(argc, argv);
		UsbWriteString("Write -h for command usage\r\n");
	}

	///Con solo el nombre y otro argumento la única opción es haber marcado -h. En caso contrario error
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(OPEN_COMM_CHANNEL);
			UsbWriteString(" [OPTIONS] [channel]\r\n");
			UsbWriteString("Open a communication channel\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n");
			UsbWriteString("\t-b\t: Open the specified broadcast channel\r\n");
			UsbWriteString("\t-u\t: Open the specified unicast channel\r\n");
			UsbWriteString("\t-m\t: Open the specified mesh channel. Mesh connection opens 3 consecutive channels directly\r\n");
			UsbWriteString("\t-p\t: Select this option to print received data on the opened channel\r\n");
			UsbWriteString("\tchannel\t: The desired channel to be opened (from 1 to 65535)\r\n\r\n");
			UsbWriteString("Usage Example:\r\n");
			UsbWriteString("\t");
			UsbWriteString(OPEN_COMM_CHANNEL);
			UsbWriteString(" -p -u 312\r\n");
			UsbWriteString("\tThis will open the 312 unicast channel and will print the received data on this channel\r\n\r\n");
		}
		else{
			error_wrong_params(argc, argv);
			UsbWriteString("Write -h for command usage\r\n");
		}
	}

	///Con el comando y 2 argumentos la opción -p nunca va a existir. Solo un tipo de canal -b, -u, -m. (Además del canal)
	else if(argc == 3){

		if (strcmp(argv[1], "-b") == 0){
			uint16_t channel = atoi(argv[2]);
			yetimote_broadcast_open(bc_comm, channel, broadcast_noprint);
			UsbWriteString("Broadcast channel ");
			UsbWriteString(argv[2]);
			UsbWriteString(" successfully opened\r\n");
		}
		else if (strcmp(argv[1], "-u") == 0){
			uint16_t channel = atoi(argv[2]);
			yetimote_unicast_open(uc_comm, channel, unicast_noprint);
			UsbWriteString("Unicast channel ");
			UsbWriteString(argv[2]);
			UsbWriteString(" successfully opened\r\n");
		}
		else if (strcmp(argv[1], "-m") == 0){
			uint16_t channel = atoi(argv[2]);
			yetimote_mesh_open(msh_comm, channel, mesh_noprint);
			UsbWriteString("Mesh channel ");
			UsbWriteString(argv[2]);
			UsbWriteString(" successfully opened\r\n");
		}
		else{
			error_wrong_params(argc, argv);
			UsbWriteString("Write -h for command usage\r\n");
		}

	}

	///Con el comando y 3 argumentos tiene que aparecer un tipo de canal y la opción -p. (Además del canal)
	else if(argc == 4){

		if (strcmp(argv[1], "-p") == 0){

			if (strcmp(argv[2], "-b") == 0){
				uint16_t channel = atoi(argv[3]);
				yetimote_broadcast_open(bc_comm, channel, broadcast_print);
				UsbWriteString("Broadcast channel ");
				UsbWriteString(argv[3]);
				UsbWriteString(" successfully opened\r\n");
			}
			else if (strcmp(argv[2], "-u") == 0){
				uint16_t channel = atoi(argv[3]);
				yetimote_unicast_open(uc_comm, channel, unicast_print);
				UsbWriteString("Unicast channel ");
				UsbWriteString(argv[3]);
				UsbWriteString(" successfully opened\r\n");
			}
			else if (strcmp(argv[2], "-m") == 0){
				uint16_t channel = atoi(argv[3]);
				yetimote_mesh_open(msh_comm, channel, mesh_print);
				UsbWriteString("Mesh channel ");
				UsbWriteString(argv[3]);
				UsbWriteString(" successfully opened\r\n");
			}
			else{
				error_wrong_params(argc, argv);
				UsbWriteString("Write -h for command usage\r\n");
			}

		}

		else if (strcmp(argv[1], "-b") == 0){

			if (strcmp(argv[2], "-p") == 0){

				uint16_t channel = atoi(argv[3]);
				yetimote_broadcast_open(bc_comm, channel, broadcast_print);
				UsbWriteString("Broadcast channel ");
				UsbWriteString(argv[3]);
				UsbWriteString(" successfully opened\r\n");
			}
			else{
				error_wrong_params(argc, argv);
				UsbWriteString("Write -h for command usage\r\n");
			}

		}
		else if (strcmp(argv[1], "-u") == 0){

			if (strcmp(argv[2], "-p") == 0){
				uint16_t channel = atoi(argv[3]);
				yetimote_unicast_open(uc_comm, channel, unicast_print);
				UsbWriteString("Unicast channel ");
				UsbWriteString(argv[3]);
				UsbWriteString(" successfully opened\r\n");
			}
			else{
				error_wrong_params(argc, argv);
				UsbWriteString("Write -h for command usage\r\n");
			}

		}
		else if (strcmp(argv[1], "-m") == 0){

			if (strcmp(argv[2], "-p") == 0){
				uint16_t channel = atoi(argv[3]);
				yetimote_mesh_open(msh_comm, channel, mesh_print);
				UsbWriteString("Mesh channel ");
				UsbWriteString(argv[3]);
				UsbWriteString(" successfully opened\r\n");
			}
			else{
				error_wrong_params(argc, argv);
				UsbWriteString("Write -h for command usage\r\n");
			}

		}
		else{
			error_wrong_params(argc, argv);
			UsbWriteString("Write -h for command usage\r\n");
		}

	}

	else{
		error_wrong_params(argc, argv);
		UsbWriteString("Write -h for command usage\r\n");
	}

	return RET_OK;
}


/**
 * @brief 			Sends a message on an opened radio channel. The message to send must be under quotes "".
 * @param argc		Number of args passed to the command
 * @param argv		Pointer to args passed to the command
 */
retval_t send_message_on_channel(uint16_t argc, char** argv){

	///Solo se ha escrito el nombre del comando sin argumentos. Da error
	if(argc == 1){
		error_no_param_command(argc, argv);
		UsbWriteString("Write -h for command usage\r\n");
	}

	///Con solo el nombre y otro argumento la única opción es haber marcado -h. En caso contrario error
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(SEND_MESSAGE_ON_CHANNEL);
			UsbWriteString(" [OPTIONS] [channel] [addr (bc, mesh)] [\"message\"]\r\n");
			UsbWriteString("Send a message on an opened channel to the specified address (only for unicast and mesh)\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n");
			UsbWriteString("\t-b\t: The channel to send the message is mesh broadcast\r\n");
			UsbWriteString("\t-u\t: The channel to send the message is mesh unicast\r\n");
			UsbWriteString("\t-m\t: The channel to send the message is mesh\r\n");
			UsbWriteString("\tchannel\t: The desired channel on which send the message (from 1 to 65535)\r\n");
			UsbWriteString("\tmessage\t: The message to send. Under quotes\r\n\r\n");
			UsbWriteString("Usage Example:\r\n");
			UsbWriteString("\t");
			UsbWriteString(SEND_MESSAGE_ON_CHANNEL);
			UsbWriteString(" -b 312 \"HELLO\"\r\n");
			UsbWriteString("\tThis will send the message HELLO on the previously opened 312 broadcast channel\r\n\r\n");
		}
		else{
			error_wrong_params(argc, argv);
			UsbWriteString("Write -h for command usage\r\n");
		}
	}

	///Con el comando y al menos 3 argumentos se envía el tipo de canal, el canal y el mensaje entre comillas
	else if(argc > 3){

		///Check the message is under quotes
		if((argv[3][0] == '\"') && (argv[argc-1][strlen(argv[argc-1])-1] == '\"')){

			uint8_t i = 0;
			uint16_t j = 0;
			uint16_t message_index = 0;
			uint16_t splitLen;

			char* message = pvPortMalloc(MAX_OUT_STR_BUFF);

			uint16_t channel = atoi(argv[2]);

			for(i=3; i<argc; i++){
				splitLen = strlen(argv[i]);
				for(j=0; j<splitLen; j++){
					if(argv[i][j] != '\"'){
						message[message_index] = argv[i][j];
						message_index ++;
					}
				}
				message[message_index] = ' ';
				message_index ++;
			}
			message[message_index-1] = '\0';

			if (strcmp(argv[1], "-b") == 0){
				yetimote_broadcast_send(bc_comm, channel, (uint8_t*) message, message_index);
				UsbWriteString("Broadcast message ");
				UsbWriteString(message);
				UsbWriteString(" successfully sent in channel ");
				UsbWriteString(argv[2]);
				UsbWriteString(" \r\n");
			}
			else{
				error_wrong_params(argc, argv);
				UsbWriteString("Write -h for command usage\r\n");
			}

			vPortFree(message);

		}
		else if((argv[4][0] == '\"') && (argv[argc-1][strlen(argv[argc-1])-1] == '\"')){
			uint8_t i = 0;
			uint16_t j = 0;
			uint16_t message_index = 0;
			uint16_t splitLen;

			uint16_t addr_dest = atoi(argv[3]);
			uint16_t channel = atoi(argv[2]);
			linkaddr_t addr;

		    addr.u16 = addr_dest;



			char* message = pvPortMalloc(MAX_OUT_STR_BUFF);


			for(i=4; i<argc; i++){
				splitLen = strlen(argv[i]);
				for(j=0; j<splitLen; j++){
					if(argv[i][j] != '\"'){
						message[message_index] = argv[i][j];
						message_index ++;
					}
				}
				message[message_index] = ' ';
				message_index ++;
			}
			message[message_index-1] = '\0';

			if (strcmp(argv[1], "-u") == 0){
			    if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
					yetimote_unicast_send(uc_comm, channel, addr, (uint8_t*) message, message_index);
					UsbWriteString("Unicast message ");
					UsbWriteString(message);
					UsbWriteString(" successfully sent in channel ");
					UsbWriteString(argv[2]);
					UsbWriteString(" to addr: ");
					UsbWriteString(argv[3]);
					UsbWriteString(" \r\n");
			    }
			}
			else if (strcmp(argv[1], "-m") == 0){
			    if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
			    	yetimote_mesh_send(msh_comm, channel, addr, (uint8_t*) message, message_index);
					UsbWriteString("Mesh message ");
					UsbWriteString(message);
					UsbWriteString(" is being sent in channel ");
					UsbWriteString(argv[2]);
					UsbWriteString(" to addr: ");
					UsbWriteString(argv[3]);
					UsbWriteString(" \r\n");
			    }
			}
			else{
				error_wrong_params(argc, argv);
				UsbWriteString("Write -h for command usage\r\n");
			}

		}
		else{
			UsbWriteString("Wrong message writen. Quotes needed\r\n");
		}

	}

	/// Error al meter los parametros
	else{
		error_wrong_params(argc, argv);
		UsbWriteString("Write -h for command usage\r\n");
	}
	return RET_OK;

}

extern osMutexId packet_delay_mutex;
extern uint32_t delay;
extern uint32_t packet_num;
extern uint32_t packet_length;
extern uint8_t* packet;

retval_t change_send_delay(uint16_t argc, char** argv){
	if(argc == 1){
		error_no_param_command(argc, argv);
		UsbWriteString("Write -h for command usage\r\n");
		}
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(CHANGE_SEND_DELAY);
			UsbWriteString(" [delay]\r\n");
			UsbWriteString("Change the delay between packets when sending unicast or mesh test streams\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n");
			UsbWriteString("Usage Example:\r\n");
			UsbWriteString("\t");
			UsbWriteString(CHANGE_SEND_DELAY);
			UsbWriteString(" 50\r\n");
			UsbWriteString("\tThis will change the delay between packets to 50 ms\r\n\r\n");
		}
		else{
			osMutexWait(packet_delay_mutex, osWaitForever);
			delay = atoi(argv[1]);
			osMutexRelease(packet_delay_mutex);
			UsbWriteString("Delay changed to");
			usb_printf(" %d\r\n", (int)delay);
		}
	}
	else{
		error_wrong_params(argc, argv);
	}
	return RET_OK;
}

retval_t change_send_packet_num(uint16_t argc, char** argv){
	if(argc == 1){
		error_no_param_command(argc, argv);
		UsbWriteString("Write -h for command usage\r\n");
		}
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(CHANGE_SEND_PACKET_NUM);
			UsbWriteString(" [packets number]\r\n");
			UsbWriteString("Change the number of packets to be sent in a mesh or unicast test stream\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n");
			UsbWriteString("Usage Example:\r\n");
			UsbWriteString("\t");
			UsbWriteString(CHANGE_SEND_PACKET_NUM);
			UsbWriteString(" 1000\r\n");
			UsbWriteString("\tThis will change the number of packets to be sent in a stream to 1000\r\n\r\n");
		}
		else{
			osMutexWait(packet_delay_mutex, osWaitForever);
			packet_num = atoi(argv[1]);
			osMutexRelease(packet_delay_mutex);
			UsbWriteString("Packet Number changed to");
			usb_printf(" %d\r\n", (int)packet_num);
		}
	}
	else{
		error_wrong_params(argc, argv);
	}
	return RET_OK;
}

extern osThreadId easysafe_unicast_thread_id;

retval_t send_test_unicast_stream(uint16_t argc, char** argv){
	if(argc == 1){
		UsbWriteString("Started sending an unicast stream\r\n");
		osThreadResume(easysafe_unicast_thread_id);
		}
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(SEND_TEST_UNICAST_STREAM);
			UsbWriteString(" [delay]\r\n");
			UsbWriteString("Starts a test unicast stream transmission\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n\r\n");
		}
		else{
			error_wrong_params(argc, argv);
		}
	}
	else{
		error_wrong_params(argc, argv);
	}
	return RET_OK;
}

extern osThreadId easysafe_mesh_thread_id;
retval_t send_test_mesh_stream(uint16_t argc, char** argv){
	if(argc == 1){
		UsbWriteString("Started sending a mesh stream\r\n");
		osThreadResume(easysafe_mesh_thread_id);
		}
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(SEND_TEST_MESH_STREAM);
			UsbWriteString("\r\n");
			UsbWriteString("Starts a test mesh stream transmission\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n\r\n");
		}
		else{
			error_wrong_params(argc, argv);
		}
	}
	else{
		error_wrong_params(argc, argv);
	}
	return RET_OK;
}


retval_t change_radio(uint16_t argc, char** argv){

	if(argc == 1){
			error_no_param_command(argc, argv);
			UsbWriteString("Write -h for command usage\r\n");
		}

		///Con solo el nombre y otro argumento la única opción es haber marcado -h. En caso contrario error
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(CHANGE_RADIO);
			UsbWriteString(" [RADIO_INTERFACE]\r\n");
			UsbWriteString("Change the radio interface used\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n");
			UsbWriteString("\t  \t: cc2500\r\n");
			UsbWriteString("\t  \t: spirit_433\r\n");
			UsbWriteString("\t  \t: spirit_868\r\n");
			UsbWriteString("Usage Example:\r\n");
			UsbWriteString("\t");
			UsbWriteString(CHANGE_RADIO);
			UsbWriteString(" cc1101 \r\n");
			UsbWriteString("\tThis will change the radio interface to the CC1101\r\n\r\n");
		}
		else if (strcmp(argv[1], "cc2500")==0){
			if(current_radio_driver != RADIO_CC2500){		//Only changes the interface if it is different
				dynamic_radio_driver.close();
				dynamic_radio_driver = NETSTACK_CC2500_RADIO;
				current_radio_driver = RADIO_CC2500;
				dynamic_radio_driver.init();
				dynamic_radio_driver.on();
				UsbWriteString("Radio Changed to CC2500\r\n");
			}

		}
		else if (strcmp(argv[1], "spirit_433")==0){
			if(current_radio_driver != RADIO_SPIRIT_433){		//Only changes the interface if it is different
				dynamic_radio_driver.close();
				dynamic_radio_driver = NETSTACK_SPIRIT_RADIO;
				current_radio_driver = RADIO_SPIRIT_433;
				dynamic_radio_driver.init();
				dynamic_radio_driver.on();
				UsbWriteString("Radio Changed to SPIRIT 433\r\n");
			}
		}
		else if (strcmp(argv[1], "spirit_868")==0){
			if(current_radio_driver != RADIO_SPIRIT_868){		//Only changes the interface if it is different
				dynamic_radio_driver.close();
				dynamic_radio_driver = NETSTACK_SPIRIT_RADIO;
				current_radio_driver = RADIO_SPIRIT_868;
				dynamic_radio_driver.init();
				dynamic_radio_driver.on();
				UsbWriteString("Radio Changed to SPIRIT 868\r\n");
			}
		}
		else{
			error_wrong_params(argc, argv);
			UsbWriteString("Write -h for command usage\r\n");
		}
	}
	else{
		error_wrong_params(argc, argv);
		UsbWriteString("Write -h for command usage\r\n");
	}
	return RET_OK;
}

retval_t change_packet_length(uint16_t argc, char** argv){
	if(argc == 1){
			error_no_param_command(argc, argv);
			UsbWriteString("Write -h for command usage\r\n");
			}
		else if(argc == 2){
			if (strcmp(argv[1], "-h") == 0){
				UsbWriteString("Usage: ");
				UsbWriteString(CHANGE_PACKET_LENGTH);
				UsbWriteString(" [PACKET LENGTH]\r\n");
				UsbWriteString("Change the packet length to be sent in a mesh or unicast test stream\r\n\r\n");
				UsbWriteString("\t-h\t: print command help\r\n");
				UsbWriteString("Usage Example:\r\n");
				UsbWriteString("\t");
				UsbWriteString(CHANGE_PACKET_LENGTH);
				UsbWriteString(" 64\r\n");
				UsbWriteString("\tThis will change the packet length to be sent in a stream to 64\r\n\r\n");
			}
			else{
				uint16_t i;
				osMutexWait(packet_delay_mutex, osWaitForever);
				packet_length = atoi(argv[1]);
				vPortFree(packet);
				packet = (uint8_t*)pvPortMalloc(packet_length*sizeof(uint8_t));
				for(i=0; i<packet_length; i++){
					packet[i] = '#';
				}
				packet[i-1] = '\0';
				osMutexRelease(packet_delay_mutex);
				UsbWriteString("Packet Length changed to");
				usb_printf(" %d\r\n",  (int)packet_length);
			}
		}
		else{
			error_wrong_params(argc, argv);
		}
	return RET_OK;
}


retval_t get_time(uint16_t argc, char** argv){
	if(argc == 1){
		usb_printf("TIME: %d/%d/%d --- %d:%d:%d\r\n", get_month_day(time_control->date),get_month(time_control->date),get_year(time_control->date)
				,get_hour(time_control->date),get_minute(time_control->date),get_second(time_control->date));
	}
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(GET_TIME);
			UsbWriteString("\r\n");
			UsbWriteString("Shows the current system time\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n\r\n");
		}
		else{
			error_wrong_params(argc, argv);
		}
	}
	else{
		error_wrong_params(argc, argv);
	}
	return RET_OK;
}
retval_t set_time(uint16_t argc, char** argv){
	if(argc == 1){
			usb_printf("TIME: %d/%d/%d --- %d:%d:%d\r\n", get_month_day(time_control->date),get_month(time_control->date),get_year(time_control->date)
					,get_hour(time_control->date),get_minute(time_control->date),get_second(time_control->date));
		}
		else if(argc == 2){
			if (strcmp(argv[1], "-h") == 0){
				UsbWriteString("Usage: ");
				UsbWriteString(SET_TIME);
				UsbWriteString(" [timestamp ms]\r\n");
				UsbWriteString("Sets the system time in ms\r\n\r\n");
				UsbWriteString("\t-h\t: print command help\r\n\r\n");
			}
			else{

				uint64_t timestamp = atoll(argv[1]);
				set_timestamp(time_control, timestamp);
				usb_printf("NEW TIME: %d/%d/%d --- %d:%d:%d\r\n", get_month_day(time_control->date),get_month(time_control->date),get_year(time_control->date)
						,get_hour(time_control->date),get_minute(time_control->date),get_second(time_control->date));
			}
		}
		else{
			error_wrong_params(argc, argv);
		}
		return RET_OK;
}
