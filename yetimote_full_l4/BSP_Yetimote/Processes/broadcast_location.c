
/****************************/
#include "rime.h"
#include "broadcast_comm.h"
#include "vcom_usb_inout.h"
#include <stdio.h>
#include <string.h>

#include "var_locator.h"
#include "cc2500.h"

#define BC_EX_CHANNEL	129    //Canal por el que se envía

extern broadcast_comm_t* bc_comm;

extern void BroadcastLocation_func(void const * argument);

void broadcast_location_recv(struct broadcast_conn *c, const linkaddr_t *from);


/****************************/
/**
 *
 * @param argument
 */
void BroadcastLocation_func(void const * argument){
	/**
	 * Enviar paquete cada 30 segundos broadcast a los nodos de las salas.
	 */

	yetimote_broadcast_open(bc_comm, BC_EX_CHANNEL, broadcast_location_recv);
	flag = 0;
	uint32_t i = 0;

	while(1){
		if (NODE_ROLE == IDENTIFIER) {
			uint8_t* data_to_send = (uint8_t*) pvPortMalloc(48);
			sprintf((char*)data_to_send, "PCK NUM: %d", (int)i);
			i++;

			yetimote_broadcast_send(bc_comm, BC_EX_CHANNEL, data_to_send, strlen((char*)data_to_send) +1);
			usb_printf("I AM THE IDENTIFIER: BC TRANSMITED \r\n");
			vPortFree(data_to_send);

		}
		osDelay(10000);
	}

}


void broadcast_location_recv(struct broadcast_conn *c, const linkaddr_t *from){
	/**
	 * Mensaje que reciben los nodos del localizador
	 * Guardo la direccion del localizador en src_addr.
	 * Si es un nodo activo el flag para que pueda transmitir unicast
	 * Si es el nodo master calculo el rssi
	 */
	if ( NODE_ROLE == NODE) {
		leds_toggle(LEDS_BLUE);
		usb_printf("I AM NODE: BC RECEIVED FROM %d.%d : %s\r\n", from->u8[0],from->u8[1], (char *)packetbuf_dataptr());
		flag = 1; //activamos el flag
		src_addr = *from;

	}
	if ( NODE_ROLE == MASTER) {
		leds_toggle(LEDS_BLUE);
		msg_rssi = cc2500_rssi();
		pream = 0xAA;
		src_addr = *from;
		node_addr = linkaddr_node_addr;

		char* sprintfString = pvPortMalloc(MAX_OUT_STR_BUFF);
		uint32_t addrByte;
		addrByte = linkaddr_node_addr.u8[0];


	//	UsbWriteString("I AM MASTER (Bytes): ");

//		usb_printf("I AM MASTER: BC RECEIVED FROM %d.%d \r\n", from->u8[0],from->u8[1]);
	//	usb_printf("RSSI RECEIVED FROM IDENTIFIER %d.%d : %d dBm\r\n", from->u8[0],from->u8[1], msg_rssi);
//		usb_printf(" \r\n");
	//	usb_printf("EL LOCALIZADOR (Bytes): %d.%d HA SIDO DETECTADO POR EL NODO MASTER CON UN RSSI DE %d dBm \r\n", from->u8[0],from->u8[1], msg_rssi);
		//usb_printf(" \r\n");

		usb_printf("%c", pream);
		usb_printf("%c%c",from->u8[0],from->u8[1]);

		//sprintf(sprintfString, "%d",(int) addrByte);
	//	UsbWriteString(sprintfString);
	//	UsbWriteString(".");
		//usb_printf("%d.", addrByte);
		usb_printf("%c%c", linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1]);

		//addrByte = linkaddr_node_addr.u8[1];
		//sprintf(sprintfString, "%d",(int) addrByte);
	//	UsbWriteString(sprintfString);
		//usb_printf("%d", addrByte);
		usb_printf("%d\r\n", msg_rssi);

	}
}
