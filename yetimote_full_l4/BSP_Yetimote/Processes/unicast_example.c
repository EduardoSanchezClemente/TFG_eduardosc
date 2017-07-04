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
 * unicast_example.c
 *
 *  Created on: 4/3/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 */


#include "yetimote-conf.h"
#include "unicast_comm.h"
#include "rime.h"
#include "vcom_usb_inout.h"

#define UC_EX_CHANNEL	139

extern unicast_comm_t* uc_comm;

extern void UnicastExample_func(void const * argument);
void unicast_example_recv(struct unicast_conn *c, const linkaddr_t *from);
/*---------------------------------------------------------------------------*/


void UnicastExample_func(void const * argument){

  yetimote_unicast_open(uc_comm, UC_EX_CHANNEL, unicast_example_recv);
  uint32_t i = 0;
  linkaddr_t dest_addr;

  dest_addr.u8[0] = 36;
  dest_addr.u8[1] = 93;

  while(1) {

    uint8_t* data_to_send = (uint8_t*) pvPortMalloc(48);
	sprintf((char*)data_to_send, "PCK NUM: %d", (int)i);
	i++;

    if(!linkaddr_cmp(&dest_addr, &linkaddr_node_addr)) {
    	 yetimote_unicast_send(uc_comm, UC_EX_CHANNEL, dest_addr, data_to_send, strlen((char*)data_to_send) +1);
    }

    vPortFree(data_to_send);
    osDelay(2000);

  }


}
/*---------------------------------------------------------------------------*/
void unicast_example_recv(struct unicast_conn *c, const linkaddr_t *from){

	usb_printf("UC MESSAGE RECEIVED FROM %d.%d : %s\r\n", from->u8[0],from->u8[1], (char *)packetbuf_dataptr());
}

