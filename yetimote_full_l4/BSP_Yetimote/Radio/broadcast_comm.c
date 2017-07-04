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
 * broadcast_comm.c
 *
 *  Created on: 28/3/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 */


#include "broadcast_comm.h"

osMutexDef(broadcastList_mutex);


/**
 * @brief	Creates a new broadcast communications list object
 * @return	The new broadcast communications list
 */
broadcast_comm_t* new_broadcast_comm(void){
	broadcast_comm_t* bc_comm = (broadcast_comm_t*) pvPortMalloc(sizeof(broadcast_comm_t));
	bc_comm->broadcastList_mutex = osMutexCreate(osMutex(broadcastList_mutex));
	bc_comm->broadcastList = gen_list_init();

	return bc_comm;
}

/**
 * @brief 			Removes an existing broadcast communications list object
 * @param bc_comm	The broadcast communications list to be removed
 * @return			Return Status [RET_OK, RET_ERROR]
 * @note			The memory reserved by the broadcast connections in the list is freed
 */
retval_t delete_broadcast_comm(broadcast_comm_t* bc_comm){
	if(bc_comm == NULL){
		return RET_ERROR;
	}
	else{
		osMutexWait(bc_comm->broadcastList_mutex, osWaitForever);
		gen_list_remove_all(bc_comm->broadcastList);
		osMutexRelease(bc_comm->broadcastList_mutex);
		osMutexDelete(bc_comm->broadcastList_mutex);
		vPortFree(bc_comm);
		return RET_OK;
	}
}


/**
 * @brief 						Open a broadcast connection
 * @param bc_comm				The broadcast communications list object
 * @param channel				Channel number of the broadcast connection
 * @param rcv_callback_func		The callback function for receiving packets
 * @return						Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_broadcast_open(broadcast_comm_t* bc_comm, uint16_t channel, void (*rcv_callback_func)(struct broadcast_conn *, const linkaddr_t *)){

	if(bc_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}
	osMutexWait(bc_comm->broadcastList_mutex, osWaitForever);

	struct broadcast_conn* broadcast = pvPortMalloc(sizeof(struct broadcast_conn));
	struct broadcast_callbacks* broadcast_callback = pvPortMalloc(sizeof(struct broadcast_callbacks));
	broadcast_callback->recv = rcv_callback_func;

	broadcast_open(broadcast, channel, broadcast_callback);
	gen_list_add(bc_comm->broadcastList, (void*) broadcast);

	osMutexRelease(bc_comm->broadcastList_mutex);
	return RET_OK;

}

/**
 * @brief				Close a broadcast connection
 * @param bc_comm		The broadcast communications list object
 * @param channel		Channel number of the broadcast connection
 * @return				Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_broadcast_close(broadcast_comm_t* bc_comm, uint16_t channel){

	if(bc_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}


	osMutexWait(bc_comm->broadcastList_mutex, osWaitForever);

	gen_list* current = bc_comm->broadcastList;
	struct broadcast_conn* broadcast;

	while(current->next != NULL){

		broadcast = (struct broadcast_conn*) current->next->item;

		if(broadcast->c.channel.channelno == channel){

			vPortFree(current->next);
			current->next = current->next->next;
			vPortFree(broadcast);
		}
		else{
			current = current->next;
		}
	}
	osMutexRelease(bc_comm->broadcastList_mutex);
	return RET_OK;

}

/**
 * @brief					Send a message on an opened broadcast connection
 * @param bc_comm			The broadcast communications list object
 * @param channel			Channel number of the broadcast connection
 * @param payload			Message to send
 * @param payload_len		Length (bytes) of the message to send
 * @return					Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_broadcast_send(broadcast_comm_t* bc_comm, uint16_t channel, uint8_t* payload, uint16_t payload_len){

	if(bc_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}

	osMutexWait(bc_comm->broadcastList_mutex, osWaitForever);

	gen_list* current = bc_comm->broadcastList;
	struct broadcast_conn* broadcast;

	while(current->next != NULL){

		broadcast = (struct broadcast_conn*) current->next->item;

		if(broadcast->c.channel.channelno == channel){

			packetbuf_copyfrom(payload, payload_len);
			packetbuf_set_attr(PACKETBUF_ATTR_RADIO_TXPOWER, 0xFF);	//0x0E -20dBm  0x12 -30dBm  0xC0 10dBm	CC1101//0xFF 1dBm CC2500
			broadcast_send(broadcast);
			break;
		}
		else{
			current = current->next;
		}
	}

	osMutexRelease(bc_comm->broadcastList_mutex);
	return RET_OK;
}


/**
 * @brief		Print the received message with the channel of the broadcast connection
 * @param c		Broadcast connection that received the message
 * @param from	Address of the message sender
 */
void broadcast_print(struct broadcast_conn *c, const linkaddr_t *from)
{
	uint8_t* packetbuffptr = (uint8_t*) packetbuf_dataptr();

	usb_printf("[%d] BC Message Received From %d.%d: \"%s\"\r\n", (int) c->c.channel.channelno, (int) from->u8[0], (int) from->u8[1] , packetbuffptr);
	UsbWriteString(">");
}

/**
 * @brief		Do nothing. Do not print anything when receives a bc message
 * @param c		Broadcast connection that received the message
 * @param from	Address of the message sender
 */
void broadcast_noprint(struct broadcast_conn *c, const linkaddr_t *from)
{
	return;
}
