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
 * config_commands.h
 *
 *  Created on: 7/6/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file config_commands.h
 */

#ifndef APPLICATION_USER_S4BIM_DSP_CODE_COMMUNICATIONS_CONFIG_COMMANDS_H_
#define APPLICATION_USER_S4BIM_DSP_CODE_COMMUNICATIONS_CONFIG_COMMANDS_H_

#include "yetimote-conf.h"

#define MAX_OUT_STR_BUFF 64		//TAMAÑO MAXIMO DEL BUFFER DE SALIDA

/**
 * COMMANDS DEFINITIONS
 */
#define LIST_CMD					"lcmd"
#define PRINT_MEM					"mem"
#define PRINT_ADDR					"addr"
#define OPEN_COMM_CHANNEL			"opench"
#define SEND_MESSAGE_ON_CHANNEL		"send"
#define CHANGE_SEND_DELAY			"delay"
#define CHANGE_SEND_PACKET_NUM		"pcktnum"
#define SEND_TEST_UNICAST_STREAM	"ucstr"
#define SEND_TEST_MESH_STREAM		"meshstr"
#define CHANGE_RADIO				"radio"
#define CHANGE_PACKET_LENGTH		"pcktlen"
#define GET_TIME					"get_time"
#define SET_TIME					"set_time"
#define CONFIG						"config"
#define ACC							"acc"
/*****************************************************/

typedef struct command_data{
	uint16_t argc;
	char** argv;
	retval_t (* command_func)(uint16_t argc, char** argv);
}command_data_t;

/**
 * FLAG FOR PRINTING OR NOT THE RECEIVED MESSAGE ON A COMMUNICATION CHANNEL
 */
typedef enum {
	NO_PRINT_RCV,//!< NO_PRINT_RCV
	PRINT_RCV,   //!< PRINT_RCV
	EASYSAFE_TEST,
	FRESQUITO_RCV,
}Print_Rcv;

////The functions or variables of these includes will be called by the commands functions////
#include "broadcast_comm.h"
#include "unicast_comm.h"
#include "mesh_comm.h"
#include "FreeRTOS.h"
#include "linkaddr.h"
#include "cmsis_os.h"
#include "netstack.h"
#include "time_control.h"


command_data_t* new_command_data();
retval_t remove_command_data(command_data_t* command_data);

retval_t select_command(command_data_t* command_data);
retval_t execute_command(command_data_t* command_data);

#endif /* APPLICATION_USER_S4BIM_DSP_CODE_COMMUNICATIONS_CONFIG_COMMANDS_H_ */
