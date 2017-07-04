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
 * main_process.c
 *
 *  Created on: 2/3/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 */


#include <Accelerometer/accelerometer_driver.h>
#include "main_process.h"

#include "rime.h"

#include "spirit1-arch.h"
#include "spirit1.h"
#include "output_packet.h"
#include "time_control.h"
#include "os_spi.h"
#include "os_i2c.h"

/* Driver objects */
#define SPI2_DRIVER		2					//SPI2 bus used to control the transceivers
spi_driver_t* spi2_driver = NULL;
//#define SPI1_DRIVER		1				//SPI1 extra bus
//spi_driver_t* spi1_driver = NULL;
#define I2C1_DRIVER		1					//I2C1 bus used to control the acelerometers
i2c_driver_t* i2c1_driver = NULL;

uint8_t btn_pressed = 0;

void adapt_Sch_Test_Pin_Callback(void);

extern time_control_t* time_control;
//extern acelerometer_driver_t* lis3dh_driver;
broadcast_comm_t* bc_comm;
unicast_comm_t* uc_comm;
mesh_comm_t* msh_comm;

void init_pattern(void);
void node_addr_init(void);
void radio_stack_init(void);
void processes_init(void);

void UsbRcvTask_callback(void const * argument);
//void BroadcastExample_func(void const * argument);
//void UnicastExample_func(void const * argument);
void BroadcastLocation_func(void const * argument);
void UnicastLocation_func(void const * argument);

//void MeshExample_func(void const * argument);
//void easysafeUnicastTest_callback(void const * argument);
//void easysafeMeshTest_callback(void const * argument);
//void data_acquisition_Task_main(void const * argument);
//void lis3dh_acquisition_Task_main(void const * argument);
void accelerometer_test_task_callback(void const * argument);


void main_process(void){		//Program starts here after init
	leds_init();	//! Inicializo estructura de leds
#ifdef	SPI2_DRIVER
	spi2_driver = new_spi_driver(SPI2_DRIVER);	//!Inicialización de los drivers SPI
	spi2_driver->spi_funcs.spi_init(spi2_driver, SPI_MASTER, LINES_DUPLEX, 8, SPI_POL_LOW, SPI_1EDGE, SPI_SOFT_NSS, 6000000, 500);
#endif
#ifdef	SPI1_DRIVER
	spi1_driver = new_spi_driver(SPI1_DRIVER);	//!Inicialización del driver SPI
	spi1_driver->spi_funcs.spi_init(spi1_driver, SPI_SLAVE, LINES_DUPLEX, 8, SPI_POL_LOW, SPI_2EDGE, SPI_SOFT_NSS, 3000000, 500);
#endif
#ifdef	I2C1_DRIVER
	i2c1_driver = new_i2c_driver(I2C1_DRIVER);	//!Inicialización del driver I2C
	i2c1_driver->i2c_funcs.i2c_init(i2c1_driver, 0 , 500);
#endif

	init_pattern();	//!	Patron de inicializacion. Esto es decoración. Eliminarse si se quiere

	node_addr_init();

	radio_stack_init();	//! Inicializar el stack radio. También se asigna la dirección física del nodo

	processes_init();	//! Inicializa los procesos a ejecutar en el nodo

	while(1){		//! Toggle led - System running
	//	usb_printf("TIME: %d/%d/%d --- %d:%d:%d\r\n", get_month_day(time_control->date),get_month(time_control->date),get_year(time_control->date)
	//			,get_hour(time_control->date),get_minute(time_control->date),get_second(time_control->date));
		osDelay(1000);
		leds_toggle(LEDS_GREEN);
	}
}

/**
 *	@brief	Blinking leds pattern and some usb output. This function dont do anything useful
 */
void init_pattern(void){

	leds_off(LEDS_ALL);
	UsbWriteString(">Initializing...\r\n");
	osDelay(150);			//Wait 200ms
	leds_on(LEDS_BLUE);
	UsbWriteString("...\r\n");
	osDelay(150);
	leds_on(LEDS_GREEN);
	UsbWriteString("...\r\n");
	osDelay(150);
	leds_on(LEDS_RED1);
	UsbWriteString("...\r\n");
	osDelay(150);
	leds_on(LEDS_RED2);
	UsbWriteString("...\r\n");
	osDelay(150);
	leds_off(LEDS_RED2);
	UsbWriteString("...\r\n");
	osDelay(150);
	leds_off(LEDS_RED1);
	UsbWriteString("...\r\n");
	osDelay(150);
	leds_off(LEDS_GREEN);
	UsbWriteString("...\r\n");
	osDelay(150);
	leds_off(LEDS_BLUE);
	UsbWriteString("...\r\n");
	UsbWriteString(">YetiMote running\r\n");
	osDelay(150);
	leds_off(LEDS_ALL);
	UsbWriteString(">");

}

void node_addr_init(void){
//SET NODE ADDRESS
	linkaddr_t addr;

	uint32_t node_id0 =  (NODEID_LOCATION_BASE+NODEID_OFFSET0)[0];
//		uint32_t node_id1 =  (NODEID_LOCATION_BASE+NODEID_OFFSET1)[0];
//		uint32_t node_id2 =  (NODEID_LOCATION_BASE+NODEID_OFFSET2)[0];

	addr.u8[0] = (uint8_t) node_id0;
	addr.u8[1] = (uint8_t) (node_id0>>16);

	linkaddr_set_node_addr(&addr);

}

void radio_stack_init(void){

	HAL_GPIO_WritePin(SPIRIT_433_SPI_CS_PORT, SPIRIT_433_SPI_CS_PIN, GPIO_PIN_SET);			//SPIRIT CS PINS
	HAL_GPIO_WritePin(SPIRIT_868_SPI_CS_PORT, SPIRIT_868_SPI_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPIRIT_433_GPIO_SDN_PORT, SPIRIT_433_GPIO_SDN_PIN, GPIO_PIN_SET);		//SPIRIT MODULES SHUTDOWN
	HAL_GPIO_WritePin(SPIRIT_868_GPIO_SDN_PORT, SPIRIT_868_GPIO_SDN_PIN, GPIO_PIN_SET);

	osDelay(1);

#if USE_RADIO
	{//ALLOCATE MEMORY FOR PACKETBUF AND QUEUEBUF
		queuebuf_init();
		packetbuf_init();
	}

	{//INIT netstack
		netstack_init();
	}
	bc_comm = new_broadcast_comm();
	uc_comm = new_unicast_comm();
	msh_comm = new_mesh_comm();

#endif
}


/**
 * @brief	Initializes the processes that must be executed on start time.
 * 			In this function new processes can be added and useless processes should be commented.
 */
void processes_init(void){

	//Proceso de recepción de entrada por USB. Permite el uso de la shell
#if DEBUG_USB
	osThreadDef(UsbRcvTask, UsbRcvTask_callback, osPriorityBelowNormal, 0, 320);
	osThreadCreate(osThread(UsbRcvTask), NULL);
#endif


	//Proceso de ejemplo de envío por broadcast_location
	osThreadDef(BroadcastLocationTask, BroadcastLocation_func, osPriorityBelowNormal, 0, 192);
	osThreadCreate(osThread(BroadcastLocationTask), NULL);
	//
	//Proceso de ejemplo de envío por unicast_location
	osThreadDef(UnicastLocationTask, UnicastLocation_func, osPriorityBelowNormal, 0, 192);
	osThreadCreate(osThread(UnicastLocationTask), NULL);
//





	//Proceso de ejemplo de envío por broadcast
//	osThreadDef(BroadcastExampleTask, BroadcastExample_func, osPriorityBelowNormal, 0, 192);
//	osThreadCreate(osThread(BroadcastExampleTask), NULL);
//
//	//Proceso de ejemplo de envío por unicast
//	osThreadDef(UnicastExampleTask, UnicastExample_func, osPriorityBelowNormal, 0, 192);
//	osThreadCreate(osThread(UnicastExampleTask), NULL);

	//Proceso de ejemplo de envío por mesh
//	osThreadDef(MeshExampleTask, MeshExample_func, osPriorityBelowNormal, 0, 256);
//	osThreadCreate(osThread(MeshExampleTask), NULL);


	//Procesos de test de easysafe
//	osThreadDef(EasysafeUnicastTestTask, easysafeUnicastTest_callback, osPriorityNormal, 0, 256);
//	osThreadCreate(osThread(EasysafeUnicastTestTask), NULL);
//
//	osThreadDef(EasysafeMeshTestTask, easysafeMeshTest_callback, osPriorityNormal, 0, 256);
//	osThreadCreate(osThread(EasysafeMeshTestTask), NULL);

//	osThreadDef(DataAdquisitionTask, data_acquisition_Task_main, osPriorityNormal, 0, 256);
//	osThreadCreate(osThread(DataAdquisitionTask), NULL);

//	osThreadDef(Lis3dhAdquisitionTask, lis3dh_acquisition_Task_main, osPriorityNormal, 0, 256);
//	osThreadCreate(osThread(Lis3dhAdquisitionTask), NULL);

//	osThreadDef(AccelerometerTestTask, accelerometer_test_task_callback, osPriorityNormal, 0, 256);
//	osThreadCreate(osThread(AccelerometerTestTask), NULL);

}

/*--------------------------------------------------------------------------*/
/* GPIO INTERRUPT ROUTINE SERVICE */
/**
 * @brief			Interrupt routine for attending different GPIOs
 * @param GPIO_Pin	Pins that have activated this interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){


	if(GPIO_Pin ==  CC2500_GDO2_IRQ_PIN	){
		cc2500_interrupt();			//A packet is sent o received
	}

	if(GPIO_Pin ==  SPIRIT_433_GPIO_IRQ){								//RRZ DESCOMENTAR PARA QUE FUNCIONE LA RADIO A 433. CON LOS NODOS ACTUALES ESTA COMPARTIDO ESTE PIN
		spirit1_interrupt_callback(); //A packet is sent o received		// CON EL DE INTERRUPCION DEL ACELEROMETRO Y SOLO SE PUEDE USAR UNO U OTRO
	}

	if(GPIO_Pin ==  SPIRIT_868_GPIO_IRQ){
		spirit1_interrupt_callback(); //A packet is sent o received
	}

	if(GPIO_Pin ==  BUTTON_PIN){	//Button interrupt
		leds_toggle(LEDS_RED1);
	}

	if(GPIO_Pin == GPIO_PIN_3){					//CS INTERRUPT PIN FOR THE TEST PROGRAM OF THE ADAPTIVE SCHEDULER
		adapt_Sch_Test_Pin_Callback();
	}

#if ACEL_USE_INT_PIN
	if(GPIO_Pin ==  ACELEROMETER_PIN){
		if (lis3dh_driver != NULL){
			lis3dh_driver->acel_funcs.acel_driver_interrupt_callback();
		}
	}
#endif
}

