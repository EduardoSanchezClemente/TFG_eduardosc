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
 * easysafe_test.c
 *
 *  Created on: 14/6/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file easysafe_test.c
 */

#include "yetimote-conf.h"

#include "leds.h"
#include "rime.h"
#include "mesh.h"
#include "route.h"
#include "mesh_comm.h"

#include <stdio.h>

#define NUM_PACKET_BAUD_RATE_CALC	20

extern unicast_comm_t* uc_comm;
extern mesh_comm_t* msh_comm;

extern void unicast_print(struct unicast_conn *c, const linkaddr_t *from);
extern void unicast_noprint(struct unicast_conn *c, const linkaddr_t *from);
extern void mesh_print(struct unicast_conn *c, const linkaddr_t *from, uint8_t hops);
extern void mesh_noprint(struct unicast_conn *c, const linkaddr_t *from, uint8_t hops);
void unicast_easysafe_rcv(struct unicast_conn *c, const linkaddr_t *from);
void mesh_easysafe_rcv(struct mesh_conn *c, const linkaddr_t *from, uint8_t hops);

extern void easysafeMeshTest_callback(void const * argument);
extern void easysafeUnicastTest_callback(void const * argument);

void easysafeMeshTestTimerFunc(void const * argument);
void easysafeUnicastTestTimerFunc(void const * argument);

osTimerId easysafeUnicastTestTimerId;
osTimerDef(easysafeUnicastTestTimer, easysafeUnicastTestTimerFunc);

osTimerId easysafeMeshTestTimerId;
osTimerDef(easysafeMeshTestTimer, easysafeMeshTestTimerFunc);

uint8_t mesh_timeout = 0;
uint8_t unicast_timeout = 0;

osThreadId easysafe_mesh_thread_id;
osThreadId easysafe_unicast_thread_id;

osMutexId packet_delay_mutex;
osMutexDef(packet_delay_mutex);
uint32_t delay = 80;
uint32_t packet_num = 100;
uint32_t packet_length = 48;
uint8_t* packet;

uint32_t easysafe_unicast_test_count = 0;
uint32_t unicast_packets_received = 0;

uint32_t easysafe_mesh_test_count = 0;
uint32_t mesh_packets_received = 0;

void easysafeMeshTest_callback(void const * argument){


	easysafe_mesh_thread_id = osThreadGetId();


	uint32_t local_delay = 80;
	uint32_t local_packet_num = 100;
	uint32_t local_packet_length = 48;
	uint32_t time1 = 0;
	uint32_t time2 = 0;
	uint32_t i;

	//STATIC ROUTING
	linkaddr_t self_addr;
	linkaddr_t dest_addr;
	linkaddr_t next_addr;

	self_addr.u8[0] = 52;
	self_addr.u8[1] = 74;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){
		dest_addr.u8[0] = 49;
		dest_addr.u8[1] = 47;

		next_addr.u8[0] = 44;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 2, 0);

		dest_addr.u8[0] = 41;
		dest_addr.u8[1] = 49;

		next_addr.u8[0] = 44;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 1, 0);

		dest_addr.u8[0] = 44;
		dest_addr.u8[1] = 49;

		next_addr.u8[0] = 44;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 0, 0);

		yetimote_mesh_open(msh_comm, 100, mesh_print);
	}


	self_addr.u8[0] = 44;
	self_addr.u8[1] = 49;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){
		dest_addr.u8[0] = 49;
		dest_addr.u8[1] = 47;

		next_addr.u8[0] = 41;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 1, 0);

		dest_addr.u8[0] = 41;
		dest_addr.u8[1] = 49;

		next_addr.u8[0] = 41;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 0, 0);

		dest_addr.u8[0] = 52;
		dest_addr.u8[1] = 74;

		next_addr.u8[0] = 52;
		next_addr.u8[1] = 74;

		route_add(&dest_addr, &next_addr, 0, 0);

		yetimote_mesh_open(msh_comm, 100, mesh_noprint);
	}


	self_addr.u8[0] = 41;
	self_addr.u8[1] = 49;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){
		dest_addr.u8[0] = 49;
		dest_addr.u8[1] = 47;

		next_addr.u8[0] = 49;
		next_addr.u8[1] = 47;

		route_add(&dest_addr, &next_addr, 0, 0);

		dest_addr.u8[0] = 44;
		dest_addr.u8[1] = 49;

		next_addr.u8[0] = 44;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 0, 0);

		dest_addr.u8[0] = 52;
		dest_addr.u8[1] = 74;

		next_addr.u8[0] = 44;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 1, 0);

		yetimote_mesh_open(msh_comm, 100, mesh_noprint);
	}


	self_addr.u8[0] = 49;
	self_addr.u8[1] = 47;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){
		dest_addr.u8[0] = 52;
		dest_addr.u8[1] = 74;

		next_addr.u8[0] = 41;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 2, 0);

		dest_addr.u8[0] = 44;
		dest_addr.u8[1] = 49;

		next_addr.u8[0] = 41;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 1, 0);

		dest_addr.u8[0] = 41;
		dest_addr.u8[1] = 49;

		next_addr.u8[0] = 41;
		next_addr.u8[1] = 49;

		route_add(&dest_addr, &next_addr, 0, 0);

		yetimote_mesh_open(msh_comm, 100, mesh_easysafe_rcv);
	}


	self_addr.u8[0] = 49;
	self_addr.u8[1] = 47;


	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){	//Receiver Task

		easysafeMeshTestTimerId = osTimerCreate (osTimer(easysafeMeshTestTimer), osTimerOnce , NULL);


		float baud = 0;



		while(1){
			while(easysafe_mesh_test_count == 0){
				osThreadSuspend(easysafe_mesh_thread_id);
			}
			osTimerStart (easysafeMeshTestTimerId, SYSTEM_CONF_SECOND*7);
			time1 = osKernelSysTick ();
			while(!mesh_timeout){
				if(easysafe_mesh_test_count >= NUM_PACKET_BAUD_RATE_CALC){
					break;
				}
				osThreadSuspend(easysafe_mesh_thread_id);
			}
			time2 = osKernelSysTick ();
			time2 = time2-time1;
			osMutexWait(packet_delay_mutex, osWaitForever);
			local_packet_length = packet_length;
			osMutexRelease(packet_delay_mutex);
			baud =	(float) local_packet_length*NUM_PACKET_BAUD_RATE_CALC*8/time2;
			usb_printf("ELAPSED TIME: %d\r\n", (int) time2);
			usb_printf("BAUD RATE: %.3f kbps\r\n", baud);
			UsbWriteString(">");

			mesh_timeout = 0;
			easysafe_mesh_test_count = 0;
		}
	}

	self_addr.u8[0] = 52;
	self_addr.u8[1] = 74;

	dest_addr.u8[0] = 49;
	dest_addr.u8[1] = 47;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){	//Transmitter task
		while(1){
			osThreadSuspend(easysafe_mesh_thread_id);

			osMutexWait(packet_delay_mutex, osWaitForever);
			local_delay = delay;
			local_packet_num = packet_num;
			local_packet_length = packet_length;


			for(i=0; i<local_packet_num; i++){
				yetimote_mesh_send(msh_comm, 100, dest_addr, (uint8_t*) packet, local_packet_length);
				osDelay(local_delay);
			}

			osMutexRelease(packet_delay_mutex);
			UsbWriteString("MESH TEST STREAM ENDED \r\n");
			UsbWriteString(">");
		}
	}

	while(1){
		osDelay(1000);
	}



}


void easysafeUnicastTest_callback(void const * argument){

	packet_delay_mutex = osMutexCreate(osMutex(packet_delay_mutex));

	easysafe_unicast_thread_id = osThreadGetId();

	uint32_t local_delay = 50;
	uint32_t local_packet_num = 500;
	uint32_t local_packet_length = 32;
	uint32_t i;

	osMutexWait(packet_delay_mutex, osWaitForever);
	packet = (uint8_t*)pvPortMalloc(packet_length*sizeof(uint8_t));
	for(i=0; i<packet_length; i++){
		packet[i] = '#';
	}
	packet[i-1] = '\0';
	osMutexRelease(packet_delay_mutex);

	//STATIC ROUTING

	linkaddr_t self_addr;
	linkaddr_t dest_addr;

	self_addr.u8[0] = 52;
	self_addr.u8[1] = 74;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){

		yetimote_unicast_open(uc_comm, 200, unicast_print);
	}


	self_addr.u8[0] = 44;
	self_addr.u8[1] = 49;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){

		yetimote_unicast_open(uc_comm, 200, unicast_noprint);
	}


	self_addr.u8[0] = 41;
	self_addr.u8[1] = 49;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){

		yetimote_unicast_open(uc_comm, 200, unicast_print);
	}


	self_addr.u8[0] = 49;
	self_addr.u8[1] = 47;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){

		yetimote_unicast_open(uc_comm, 200, unicast_easysafe_rcv);
	}

	self_addr.u8[0] = 49;
	self_addr.u8[1] = 47;

	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){	//Receiver Task

		easysafeUnicastTestTimerId = osTimerCreate (osTimer(easysafeUnicastTestTimer), osTimerOnce , NULL);
		uint32_t time1 = 0;
		uint32_t time2 = 0;

		float baud = 0;

		while(1){
			while(easysafe_unicast_test_count == 0){
				osThreadSuspend(easysafe_unicast_thread_id);
			}
			osTimerStart (easysafeUnicastTestTimerId, SYSTEM_CONF_SECOND*7);
			time1 = osKernelSysTick ();
			while(!unicast_timeout){
				if(easysafe_unicast_test_count >= NUM_PACKET_BAUD_RATE_CALC){
					break;
				}
				osThreadSuspend(easysafe_unicast_thread_id);
			}

			time2 = osKernelSysTick ();
			time2 = time2-time1;
			osMutexWait(packet_delay_mutex, osWaitForever);
			local_packet_length = packet_length;
			osMutexRelease(packet_delay_mutex);
			baud =	(float) local_packet_length*NUM_PACKET_BAUD_RATE_CALC*8/time2;
			usb_printf("ELAPSED TIME: %d\r\n", (int) time2);
			usb_printf("BAUD RATE: %.3f kbps\r\n", baud);
			UsbWriteString(">");

			unicast_timeout = 0;
			easysafe_unicast_test_count = 0;
		}
	}

	self_addr.u8[0] = 52;
	self_addr.u8[1] = 74;

	dest_addr.u8[0] = 49;
	dest_addr.u8[1] = 47;


	if(linkaddr_cmp(&self_addr, &linkaddr_node_addr)){	//Transmitter task
		while(1){
			osThreadSuspend(easysafe_unicast_thread_id);

			osMutexWait(packet_delay_mutex, osWaitForever);
			local_delay = delay;
			local_packet_num = packet_num;
			local_packet_length = packet_length;


			for(i=0; i<local_packet_num; i++){
				yetimote_unicast_send(uc_comm, 200, dest_addr, (uint8_t*) packet, local_packet_length);
				osDelay(local_delay);
			}

			osMutexRelease(packet_delay_mutex);

			UsbWriteString("UNICAST TEST STREAM ENDED \r\n");
			UsbWriteString(">");

		}
	}


	while(1){
		osDelay(1000);
	}



}

/**
 * @brief		Do nothing. Do not print anything when receives a uc message
 * @param c		mesh connection that received the message
 * @param from	Address of the message sender
 * @param hops	Number of hops of the message
 */
void unicast_easysafe_rcv(struct unicast_conn *c, const linkaddr_t *from){

	uint8_t* packetbuffptr = (uint8_t*) packetbuf_dataptr();
//	usb_printf("[%d] EASYSAFE UC Received From %d.%d: \"%s\"\r\n", (int) c->c.c.channel.channelno, (int) from->u8[0], (int) from->u8[1] , packetbuffptr);
//	UsbWriteString(">");
	if(strcmp((char*) packetbuffptr,(char*) packet) == 0){		//64 bytes including \0
		easysafe_unicast_test_count++;
		unicast_packets_received++;
		osThreadResume(easysafe_unicast_thread_id);
	}
	return;
}

/**
 * @brief		Do nothing. Do not print anything when receives a uc message
 * @param c		mesh connection that received the message
 * @param from	Address of the message sender
 * @param hops	Number of hops of the message
 */
void mesh_easysafe_rcv(struct mesh_conn *c, const linkaddr_t *from, uint8_t hops){

	uint8_t* packetbuffptr = (uint8_t*) packetbuf_dataptr();
//	usb_printf("[%d] EASYSAFE Message Received From %d.%d: \"%s\"\r\n", (int) c->multihop.c.c.c.channel.channelno, (int) from->u8[0], (int) from->u8[1] , packetbuffptr);
//	UsbWriteString(">");
	if(strcmp((char*) packetbuffptr,(char*) packet) == 0){		//64 bytes including \0
		easysafe_mesh_test_count++;
		mesh_packets_received++;
		osThreadResume(easysafe_mesh_thread_id);
	}
	return;
}

void easysafeUnicastTestTimerFunc(void const * argument){
	unicast_timeout = 1;
	usb_printf("Total Unicast packets received: %d\r\n", (int)unicast_packets_received);
	UsbWriteString(">");
	unicast_packets_received = 0;
	osThreadResume(easysafe_unicast_thread_id);
}

void easysafeMeshTestTimerFunc(void const * argument){
	mesh_timeout = 1;
	usb_printf("Total Mesh packets received: %d\r\n", (int)mesh_packets_received);
	UsbWriteString(">");
	mesh_packets_received = 0;
	osThreadResume(easysafe_mesh_thread_id);
}

