/*
 * UsbRcvTask.c
 *
 *  Created on: 2/3/2016
 *      Author: Yo
 */

#include "leds.h"
#include "vcom_usb_inout.h"
#include "yetimote-conf.h"
#include "ext_comm_input.h"

#define RCV_UPDATE_TIMEOUT SYSTEM_CONF_SECOND*3

osThreadId taskId = NULL;
input_data_t* input_data;

extern USBD_HandleTypeDef hUsbDeviceFS;

extern void  UsbRcvTask_callback(void const * argument);
void usbRcvTimeout(void const * argument);

/**
 * @brief			This task process incoming data from USB (or other interfaces) and executes a command if \r or \n is present
 * @param argument	No argument used
 */
void UsbRcvTask_callback(void const * argument){

	taskId = osThreadGetId();

	osTimerDef(usbRcvTimer, usbRcvTimeout);
	osTimerId usbRcvTimerId = osTimerCreate (osTimer(usbRcvTimer), osTimerPeriodic , NULL);

	input_data = new_input_data(MAX_CMD_INPUT_BUFFER);

	osTimerStart (usbRcvTimerId, RCV_UPDATE_TIMEOUT);	//A veces se cuelga y el USB deja de recibir. Si pasado el tiempo RCV_UPDATE_TIMEOUT no se ha recibido nada por usb, se vuelve a poner el usb a escuchar.

	while(1){

		get_new_data(input_data);
		check_newline(input_data);
		osMutexWait (input_data->circular_buffer_mutex, SYSTEM_CONF_SECOND/10);	//Protect access to input_data->circular_buffer
		select_command(input_data->command);
		execute_command(input_data->command);
		osMutexRelease (input_data->circular_buffer_mutex);

		osDelay(20);
	}

}

//A esta funcion se llama directamente desde la interrupcion al recibir un paquete USB

void VirtualCom_Rcv_Callback(uint8_t *Buf, uint32_t Len){	//No bloquear esta funcion. Se llama desde una interrupcion
															//Simplemente guardar el buffer recibido para usarlo como convenga
															//Tampoco llamar a printf ni nada similar, ya que usa interrupciones y se bloquea.

	if(taskId != NULL){			//Si la tarea UsbRcvTask no ha empezado todavía no hago nada
		uint16_t i = 0;

		for(i=0; i<Len; i++){
			input_data->circular_buffer[CIRCULAR_BUFFER_SIZE-input_data->rcv_usb_buffer_counter] = Buf[i];
			input_data->rcv_usb_buffer_counter--;
			if(input_data->rcv_usb_buffer_counter <= 0){
				input_data->rcv_usb_buffer_counter = CIRCULAR_BUFFER_SIZE;
			}
			USBD_CDC_ReceivePacket(&hUsbDeviceFS);		//Esta instrucción indica al driver USB que ya esta listo para recibir más paquetes, ya que el dato anterior ya se ha procesado
		}
	}

}


void usbRcvTimeout(void const * argument){
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}
