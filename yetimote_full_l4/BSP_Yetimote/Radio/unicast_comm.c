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
 * unicast_comm.c
 *
 *  Created on: 22/4/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 */


#include "unicast_comm.h"

osMutexDef(unicastList_mutex);

/**
 * @brief 	Creates a new unicast communications list object
 * @return	The new unicast communications list
 */
unicast_comm_t* new_unicast_comm(void){
	unicast_comm_t* uc_comm = (unicast_comm_t*) pvPortMalloc(sizeof(unicast_comm_t));
	uc_comm->unicastList_mutex = osMutexCreate(osMutex(unicastList_mutex));
	uc_comm->unicastList = gen_list_init();

	return uc_comm;
}

/**
 * @brief			Removes an existing unicast communications list object
 * @param uc_comm	The unicast list to be removed
 * @return			Return Status [RET_OK, RET_ERROR]
 * @note			The memory reserved by the unicast connections in the list is freed
 */
retval_t delete_unicast_comm(unicast_comm_t* uc_comm){
	if(uc_comm == NULL){
		return RET_ERROR;
	}
	else{
		osMutexWait(uc_comm->unicastList_mutex, osWaitForever);
		gen_list_remove_all(uc_comm->unicastList);
		osMutexRelease(uc_comm->unicastList_mutex);
		osMutexDelete(uc_comm->unicastList_mutex);
		vPortFree(uc_comm);
		return RET_OK;
	}
}


/**
 * @brief 					Open an unicast connection
 * @param uc_comm			The unicast communications list object
 * @param channel			Channel number of the unicast connection
 * @param rcv_callback_func	The callback function for receiving packets
 * @return					Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_unicast_open(unicast_comm_t* uc_comm, uint16_t channel, void (*rcv_callback_func)(struct unicast_conn *, const linkaddr_t *)){

	if(uc_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}

	osMutexWait(uc_comm->unicastList_mutex, osWaitForever);

	struct unicast_conn* unicast = pvPortMalloc(sizeof(struct unicast_conn));
	struct unicast_callbacks* unicast_callback = pvPortMalloc(sizeof(struct unicast_callbacks));
	unicast_callback->recv = rcv_callback_func;

	unicast_open(unicast, channel, unicast_callback);
	gen_list_add(uc_comm->unicastList, (void*) unicast);

	osMutexRelease(uc_comm->unicastList_mutex);
	return RET_OK;

}

/**
 * @brief				Close a unicast connection
 * @param uc_comm		The unicast communications list object
 * @param channel		Channel number of the unicast connection
 * @return				Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_unicast_close(unicast_comm_t* uc_comm, uint16_t channel){

	if(uc_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}

	osMutexWait(uc_comm->unicastList_mutex, osWaitForever);

	gen_list* current = uc_comm->unicastList;
	struct unicast_conn* unicast;

	while(current->next != NULL){

		 unicast = (struct  unicast_conn*) current->next->item;

		if(unicast->c.c.channel.channelno == channel){

			vPortFree(current->next);
			current->next = current->next->next;
			vPortFree(unicast);
		}
		else{
			current = current->next;
		}
	}
	osMutexRelease(uc_comm->unicastList_mutex);
	return RET_OK;

}

/**
 * @brief				Send a message on an opened unicast connection
 * @param uc_comm		The unicast communications list object
 * @param channel		Channel number of the unicast connection
 * @param addr			Address of the unicast message destination
 * @param payload		Message to send
 * @param payload_len	Length (bytes) of the message to send
 * @return				Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_unicast_send(unicast_comm_t* uc_comm, uint16_t channel, linkaddr_t addr, uint8_t* payload, uint16_t payload_len){

	if(uc_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}

	osMutexWait(uc_comm->unicastList_mutex, osWaitForever);

	gen_list* current = uc_comm->unicastList;
	struct unicast_conn* unicast;

	while(current->next != NULL){

		unicast = (struct unicast_conn*) current->next->item;

		if(unicast->c.c.channel.channelno == channel){

			packetbuf_copyfrom(payload, payload_len);
			packetbuf_set_attr(PACKETBUF_ATTR_RADIO_TXPOWER, 0xFF);	//0x0E -20dBm  0x12 -30dBm  0xC0 10dBm	CC1101//0xFF 1dBm CC2500
			unicast_send(unicast, &addr);
			break;
		}
		else{
			current = current->next;
		}
	}

	osMutexRelease(uc_comm->unicastList_mutex);
	return RET_OK;
}

/**
 * @brief		Print the received message with the channel of the unicast connection
 * @param c		Unicast connection that received the message
 * @param from	Address of the message sender
 */
void unicast_print(struct unicast_conn *c, const linkaddr_t *from){

	uint8_t* packetbuffptr = (uint8_t*) packetbuf_dataptr();

	usb_printf("[%d] UC Message Received From %d.%d: \"%s\"\r\n", (int) c->c.c.channel.channelno, (int) from->u8[0], (int) from->u8[1] , packetbuffptr);
	UsbWriteString(">");
}

/**
 * @brief		Do nothing. Do not print anything when receives a uc message
 * @param c		Unicast connection that received the message
 * @param from	Address of the message sender
 */
void unicast_noprint(struct unicast_conn *c, const linkaddr_t *from){
	return;
}
