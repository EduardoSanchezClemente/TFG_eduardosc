
#include "yetimote-conf.h"
#include "unicast_comm.h"
#include "rime.h"
#include "vcom_usb_inout.h"

#include "var_locator.h"
#include "cc2500.h"

#define UC_EX_CHANNEL	139

extern unicast_comm_t* uc_comm;

extern void UnicastLocation_func(void const * argument);
void unicast_location_recv(struct unicast_conn *c, const linkaddr_t *from);
/*---------------------------------------------------------------------------*/



void UnicastLocation_func(void const * argument){
/**
 * Los nodos envian un mensaje unicast al nodo master con la potencia RSSI y con el ADDRESS
 * de la placa(otro nodo) que recibieron del mensaje broadcast.
 *
 * void const * argument Al declarar su parámetro como un puntero a const, esta
 * función indica que sus datos no se copiarán, solo se leerán
 */

  yetimote_unicast_open(uc_comm, UC_EX_CHANNEL, unicast_location_recv);
  linkaddr_t dest_addr;		 //Direccion del nodo master

  dest_addr.u8[0] = 78;      //Direccion del nodo master(mirado 78)
  dest_addr.u8[1] = 57;      //Direccion del nodo master(mirado 57)
  while(1) {
	  if (flag == 1){   //flag del broadcast para mandar el unicast
		  flag = 0;
		  msg_rssi = cc2500_rssi();
		  uint8_t* data_to_send = (uint8_t*) pvPortMalloc(48);  // tengo que enviar lo recibido de broadcast_location_recv
		  memcpy(data_to_send , &msg_rssi, sizeof(int));
		  memcpy(data_to_send + sizeof(int), &src_addr, sizeof(linkaddr_t));

		  if(!linkaddr_cmp(&dest_addr, &linkaddr_node_addr)) {
			yetimote_unicast_send(uc_comm, UC_EX_CHANNEL, dest_addr, data_to_send, strlen((char*)data_to_send) +1);
		  }

		  char* sprintfString = pvPortMalloc(MAX_OUT_STR_BUFF);
		  uint32_t addrByte;
		  addrByte = src_addr.u8[0];
		  sprintf(sprintfString, "%d",(int) addrByte);
		  UsbWriteString("RSSI FROM IDENTIFIER (Bytes): ");
		  UsbWriteString(sprintfString);
		  UsbWriteString(".");
		  addrByte = src_addr.u8[1];
		  sprintf(sprintfString, "%d",(int) addrByte);
		  UsbWriteString(sprintfString);
		  usb_printf(" TO SEND: %d dBm\r\n", msg_rssi);
		  usb_printf(" \r\n");
		  usb_printf("I AM NODE: UC MESSAGE TRANSMITED TO MASTER 78.57 \r\n");
		  usb_printf(" \r\n");

		  vPortFree(data_to_send);
	  }
	  osDelay(2000);

  }
}


/*---------------------------------------------------------------------------*/
void unicast_location_recv(struct unicast_conn *c, const linkaddr_t *from){
	/**
	 * El nodo master recibe la informacion de la placa desde los otros nodos de las salas
	 */
	uint8_t* packetbuffptr = (uint8_t*) packetbuf_dataptr();
	if (NODE_ROLE == MASTER ){
		pream = 0xAA;
		memcpy(&msg_rssi, packetbuffptr , sizeof(int));
		memcpy(&src_addr, packetbuffptr + sizeof(int), sizeof(linkaddr_t));
		node_addr = *from;
		char* sprintfString = pvPortMalloc(MAX_OUT_STR_BUFF);
	//	usb_printf("I AM MASTER: UC MESSAGE RECEIVED FROM NODE %d.%d \r\n", from->u8[0],from->u8[1]);

		uint32_t addrByte;
		addrByte = src_addr.u8[0];
	//	UsbWriteString("EL LOCALIZADOR (Bytes): ");

	//	usb_printf(" HA SIDO DETECTADO POR EL NODO (Bytes): %d.%d CON UN RSSI DE %d dBm \r\n", from->u8[0],from->u8[1], msg_rssi);
	//	usb_printf(" \r\n");
	//	usb_printf("%d",pream);
	//   sprintf(sprintfString, "%d",(int) addrByte);
	//    UsbWriteString(sprintfString);
	//    UsbWriteString(".");
	//    addrByte = src_addr.u8[1];
	//    sprintf(sprintfString, "%d",(int) addrByte);
	//    UsbWriteString(sprintfString);
    //	usb_printf(" %d.%d %d\r\n", from->u8[0],from->u8[1], msg_rssi);
		usb_printf("%c", pream);
		usb_printf("%c%c",src_addr.u8[0],src_addr.u8[1]);
		usb_printf("%c%c",from->u8[0],from->u8[1]);
		usb_printf("%d\r\n", msg_rssi);

	}

}

