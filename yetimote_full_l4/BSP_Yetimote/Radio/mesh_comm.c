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
 * mesh_comm.c
 *
 *  Created on: 9/5/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file mesh_comm.c
 */

#include "mesh_comm.h"

osMutexDef(meshList_mutex);

/**
 * @brief	Creates a new mesh communications list object
 * @return	The new mesh communications list
 */
mesh_comm_t* new_mesh_comm(void){
	mesh_comm_t* msh_comm = (mesh_comm_t*) pvPortMalloc(sizeof(mesh_comm_t));
	msh_comm->meshList_mutex = osMutexCreate(osMutex(meshList_mutex));
	msh_comm->meshList = gen_list_init();

	return msh_comm;
}

/**
 * @brief			Removes an existing mesh communications list object
 * @param msh_comm	The mesh list to be removed
 * @return			Return Status [RET_OK, RET_ERROR]
 * @note			The memory reserved by the mesh connections in the list is freed
 */
retval_t delete_mesh_comm(mesh_comm_t* msh_comm){
	if(msh_comm == NULL){
		return RET_ERROR;
	}
	else{
		osMutexWait(msh_comm->meshList_mutex, osWaitForever);
		gen_list_remove_all(msh_comm->meshList);
		osMutexRelease(msh_comm->meshList_mutex);
		osMutexDelete(msh_comm->meshList_mutex);
		vPortFree(msh_comm);
		return RET_OK;
	}
}

/**
 * @brief 						Open an mesh connection
 * @param msh_comm				The mesh communications list object
 * @param channel				Channel number of the mesh connection
 * @param rcv_callback_func		The callback function for receiving packets
 * @return						Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_mesh_open(mesh_comm_t* msh_comm, uint16_t channel, void (*rcv_callback_func)(struct mesh_conn *, const linkaddr_t *, uint8_t)){

	if(msh_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}

	osMutexWait(msh_comm->meshList_mutex, osWaitForever);

	struct mesh_conn* mesh = pvPortMalloc(sizeof(struct mesh_conn));
	struct mesh_callbacks* mesh_callback = pvPortMalloc(sizeof(struct mesh_callbacks));
	mesh_callback->recv = rcv_callback_func;

	mesh_open(mesh, channel, mesh_callback);
	gen_list_add(msh_comm->meshList, (void*) mesh);

	osMutexRelease(msh_comm->meshList_mutex);
	return RET_OK;
}

/**
 * @brief				Close an mesh connection
 * @param msh_comm		The mesh communications list object
 * @param channel		Channel number of the mesh connection
 * @return				Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_mesh_close(mesh_comm_t* msh_comm, uint16_t channel){

	if(msh_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}

	osMutexWait(msh_comm->meshList_mutex, osWaitForever);

	gen_list* current = msh_comm->meshList;
	struct mesh_conn* mesh;

	while(current->next != NULL){

		mesh = (struct  mesh_conn*) current->next->item;

		if(mesh->multihop.c.c.c.channel.channelno == channel){

			vPortFree(current->next);
			current->next = current->next->next;
			vPortFree(mesh);
		}
		else{
			current = current->next;
		}
	}
	osMutexRelease(msh_comm->meshList_mutex);
	return RET_OK;
}

/**
 * @brief					Send a message on an opened mesh connection
 * @param msh_comm			The mesh communications list object
 * @param channel			Channel number of the mesh connection
 * @param addr				Address of the mesh message destination
 * @param payload			Message to send
 * @param payload_len		Length (bytes) of the message to send
 * @return					Return Status [RET_OK, RET_ERROR]
 */
retval_t yetimote_mesh_send(mesh_comm_t* msh_comm, uint16_t channel, linkaddr_t addr, uint8_t* payload, uint16_t payload_len){

	if(msh_comm == NULL){
		return RET_ERROR;
	}
	if(channel < 0 || channel >65535){
		return RET_ERROR;
	}

	osMutexWait(msh_comm->meshList_mutex, osWaitForever);

	gen_list* current = msh_comm->meshList;
	struct mesh_conn* mesh;

	while(current->next != NULL){

		mesh = (struct mesh_conn*) current->next->item;

		if(mesh->multihop.c.c.c.channel.channelno == channel){

			packetbuf_copyfrom(payload, payload_len);
			packetbuf_set_attr(PACKETBUF_ATTR_RADIO_TXPOWER, 0xFF);	//0x0E -20dBm  0x12 -30dBm  0xC0 10dBm	CC1101//0xFF 1dBm CC2500
			mesh_send(mesh, &addr);
			break;
		}
		else{
			current = current->next;
		}
	}

	osMutexRelease(msh_comm->meshList_mutex);
	return RET_OK;
}


/**
 * @brief		Print the received message with the channel of the mesh connection
 * @param c		mesh connection that received the message
 * @param from	Address of the message sender
 * @param hops	Number of hops of the message
 */
void mesh_print(struct mesh_conn *c, const linkaddr_t *from, uint8_t hops)
{
	uint8_t* packetbuffptr = (uint8_t*) packetbuf_dataptr();

	usb_printf("[%d] MESH Message Received From %d.%d: \"%s\"\r\n", (int) c->multihop.c.c.c.channel.channelno, (int) from->u8[0], (int) from->u8[1] , packetbuffptr);
	UsbWriteString(">");
}

/**
 * @brief		Do nothing. Do not print anything when receives a uc message
 * @param c		mesh connection that received the message
 * @param from	Address of the message sender
 * @param hops	Number of hops of the message
 */
void mesh_noprint(struct mesh_conn *c, const linkaddr_t *from, uint8_t hops)
{
	return;
}
