
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VAR_LOCATOR_H
#define __VAR_LOCATOR_H

enum node_role {
	NODE,
	IDENTIFIER,
	MASTER
};

#define NODE_ROLE IDENTIFIER     //MODIFICAR PARA PROGRAMAR CADA TIPO DE NODO.

uint8_t flag;
linkaddr_t src_addr;                 //Address del localizador
linkaddr_t node_addr;                //Address del nodo

int msg_rssi;                        //Información del RSSI
uint8_t pream;

#endif


