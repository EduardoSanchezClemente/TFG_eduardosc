/*
 * Copyright (c) 2015, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * broadcast_example.c
 *
 *  Created on: 4/3/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 */


/****************************/
#include "rime.h"
#include "broadcast_comm.h"
#include "vcom_usb_inout.h"
#include <stdio.h>
#include <string.h>


#define BC_EX_CHANNEL	129

extern broadcast_comm_t* bc_comm;

extern void BroadcastExample_func(void const * argument);

void broadcast_example_recv(struct broadcast_conn *c, const linkaddr_t *from);


/****************************/
/**
 *
 * @param argument
 */
void BroadcastExample_func(void const * argument){

	yetimote_broadcast_open(bc_comm, BC_EX_CHANNEL, broadcast_example_recv);

	uint32_t i = 0;

	while(1){
		uint8_t* data_to_send = (uint8_t*) pvPortMalloc(48);
		sprintf((char*)data_to_send, "PCK NUM: %d", (int)i);
		i++;

		yetimote_broadcast_send(bc_comm, BC_EX_CHANNEL, data_to_send, strlen((char*)data_to_send) +1);

		vPortFree(data_to_send);
		osDelay(500);

	}

}


void broadcast_example_recv(struct broadcast_conn *c, const linkaddr_t *from){

	leds_toggle(LEDS_BLUE);
	usb_printf("BC RECEIVED FROM %d.%d : %s\r\n", from->u8[0],from->u8[1], (char *)packetbuf_dataptr());
}
