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
 * main_app.c
 *
 *  Created on: 21/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file main_app.c
 */

#include "cmsis_os.h"
#include "spi.h"
#include "gpio.h"
#include "arm_math.h"
#include "test_apps.h"
#include "os_spi.h"

uint32_t avg_delay[TASKS_NUMBER];
uint32_t num_counts[TASKS_NUMBER];
uint32_t max_delay[TASKS_NUMBER];
uint32_t min_delay[TASKS_NUMBER];

uint32_t delayed[TASKS_NUMBER];

//uint8_t received_spi = 0;

extern spi_driver_t* spi1_driver;
int16_t spi_test_id;

void init_delays(void);
void send_data(void);
void start_task_set(void);
void resume_task_set(void);
void suspend_task_set(void);

osThreadId mainTaskHandle;

//TASK SET
osThreadId t1TaskHandle;
void t1Task(void const * argument);
osThreadId t2TaskHandle;
void t2Task(void const * argument);
osThreadId t3TaskHandle;
void t3Task(void const * argument);
osThreadId t4TaskHandle;
void t4Task(void const * argument);
osThreadId t5TaskHandle;
void t5Task(void const * argument);
osThreadId t6TaskHandle;
void t6Task(void const * argument);
osThreadId t7TaskHandle;
void t7Task(void const * argument);
osThreadId t8TaskHandle;
void t8Task(void const * argument);
osThreadId t9TaskHandle;
void t9Task(void const * argument);
osThreadId t10TaskHandle;
void t10Task(void const * argument);
osThreadId t11TaskHandle;
void t11Task(void const * argument);
osThreadId t12TaskHandle;
void t12Task(void const * argument);
osThreadId t13TaskHandle;
void t13Task(void const * argument);
osThreadId t14TaskHandle;
void t14Task(void const * argument);
osThreadId t15TaskHandle;
void t15Task(void const * argument);
osThreadId t16TaskHandle;
void t16Task(void const * argument);

void main_app(void){
	uint32_t i;

//	set_bat(1);				//Test
//	set_rssi(0);
//	set_net(0);
//	set_mult(0.75f);
//	double net_var = 0;
//	while(1){
//		HAL_Delay(500);
//		net_var += 0.05f;
//		set_net(net_var);
//	}

	spi_test_id = spi1_driver->spi_funcs.spi_open(spi1_driver->spi_vars, DISABLE_SW_CS, 0, 0);

	float32_t* spi_rcv;
	float32_t* spi_tx;
	uint32_t rcv_command;
	mainTaskHandle = osThreadGetId();
	for(i=0; i<TASKS_NUMBER; i++){
		delayed[i] = 0;
	}
	start_task_set();
	osDelay(200);

	while(1){

		osThreadSuspend(mainTaskHandle);
		taskENTER_CRITICAL();
		spi_rcv = (float32_t*) pvPortMalloc(COMMAND_SIZE*sizeof(float32_t));
		spi_tx = (float32_t*) pvPortMalloc(COMMAND_SIZE*sizeof(float32_t));
		taskEXIT_CRITICAL();

		port_state_t spi_state = spi1_driver->spi_funcs.spi_port_get_state(spi1_driver->spi_vars, spi_test_id);
		if(spi_state != DRIVER_PORT_CLOSED){//RCV	COMMAND
			spi1_driver->spi_funcs.spi_read_write_nb(spi1_driver->spi_vars, spi_test_id,(uint8_t *) spi_tx, (uint8_t *) spi_rcv, COMMAND_SIZE*sizeof(float32_t));
		}
//		HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t*) spi_tx, (uint8_t*) spi_rcv, COMMAND_SIZE*sizeof(float32_t));
//		while(!received_spi){
//
//		}
//		received_spi = 0;
		rcv_command = (uint32_t) spi_rcv[0];

		switch(rcv_command){
		case START_GLOBAL_TEST:
			set_bat(spi_rcv[BAT+1]);
			set_rssi(spi_rcv[RSSI+1]);
			set_net(spi_rcv[NET+1]);
			set_mult(spi_rcv[MULT_FACTOR+1]);
			init_delays();
			enable_adapt_duty_cycle();
			resume_task_set();
			break;
		case START_NEXT:
			set_bat(spi_rcv[BAT+1]);
			set_rssi(spi_rcv[RSSI+1]);
			set_net(spi_rcv[NET+1]);
			set_mult(spi_rcv[MULT_FACTOR+1]);
			init_delays();
			enable_adapt_duty_cycle();
			resume_task_set();
			break;
		case TIMEOUT:
			suspend_task_set();
			send_data();	//Envio de datos por SPI. 3 por hebra. MAX DELAY, MIN DELAY AVG DELAY
			break;
		case INTERRUPT_TEST:
			suspend_task_set();
			break;
		case END_TEST:
			suspend_task_set();
			break;
		default:
			break;
		}
		vPortFree(spi_rcv);
		vPortFree(spi_tx);
	}

}

void start_task_set(void){
	  osThreadDef(t1Task, t1Task, VERY_LOW_PRIORITY, 0, 128);
	  t1TaskHandle = osThreadCreate(osThread(t1Task), NULL);

	  osThreadDef(t2Task, t2Task, BELOW_NORMAL_PRIORITY, 0, 128);
	  t2TaskHandle = osThreadCreate(osThread(t2Task), NULL);

	  osThreadDef(t3Task, t3Task, NORMAL_PRIORITY, 0, 128);
	  t3TaskHandle = osThreadCreate(osThread(t3Task), NULL);

	  osThreadDef(t4Task, t4Task, ABOVE_NORMAL_PRIORITY, 0, 128);
	  t4TaskHandle = osThreadCreate(osThread(t4Task), NULL);

	  osThreadDef(t5Task, t5Task, HIGH_PRIORITY, 0, 128);
	  t5TaskHandle = osThreadCreate(osThread(t5Task), NULL);

	  osThreadDef(t6Task, t6Task, REAL_TIME_PRIORITY, 0, 128);
	  t6TaskHandle = osThreadCreate(osThread(t6Task), NULL);

	  osThreadDef(t7Task, t7Task, VERY_LOW_PRIORITY, 0, 128);
	  t7TaskHandle = osThreadCreate(osThread(t7Task), NULL);

	  osThreadDef(t8Task, t8Task, BELOW_NORMAL_PRIORITY, 0, 128);
	  t8TaskHandle = osThreadCreate(osThread(t8Task), NULL);

	  osThreadDef(t9Task, t9Task, NORMAL_PRIORITY, 0, 128);
	  t9TaskHandle = osThreadCreate(osThread(t9Task), NULL);

	  osThreadDef(t10Task, t10Task, ABOVE_NORMAL_PRIORITY, 0, 128);
	  t10TaskHandle = osThreadCreate(osThread(t10Task), NULL);

	  osThreadDef(t11Task, t11Task, HIGH_PRIORITY, 0, 128);
	  t11TaskHandle = osThreadCreate(osThread(t11Task), NULL);

	  osThreadDef(t12Task, t12Task, REAL_TIME_PRIORITY, 0, 128);
	  t12TaskHandle = osThreadCreate(osThread(t12Task), NULL);

	  osThreadDef(t13Task, t13Task, VERY_LOW_PRIORITY, 0, 128);
	  t13TaskHandle = osThreadCreate(osThread(t13Task), NULL);

	  osThreadDef(t14Task, t14Task, BELOW_NORMAL_PRIORITY, 0, 128);
	  t14TaskHandle = osThreadCreate(osThread(t14Task), NULL);

	  osThreadDef(t15Task, t15Task, NORMAL_PRIORITY, 0, 128);
	  t15TaskHandle = osThreadCreate(osThread(t15Task), NULL);

	  osThreadDef(t16Task, t16Task, ABOVE_NORMAL_PRIORITY, 0, 128);
	  t16TaskHandle = osThreadCreate(osThread(t16Task), NULL);
}

void resume_task_set(void){
	osThreadResume(t1TaskHandle);
	osThreadResume(t2TaskHandle);
	osThreadResume(t3TaskHandle);
	osThreadResume(t4TaskHandle);
	osThreadResume(t5TaskHandle);
	osThreadResume(t6TaskHandle);
	osThreadResume(t7TaskHandle);
	osThreadResume(t8TaskHandle);
	osThreadResume(t9TaskHandle);
	osThreadResume(t1TaskHandle);
	osThreadResume(t10TaskHandle);
	osThreadResume(t11TaskHandle);
	osThreadResume(t12TaskHandle);
	osThreadResume(t13TaskHandle);
	osThreadResume(t14TaskHandle);
	osThreadResume(t15TaskHandle);
	osThreadResume(t16TaskHandle);
}

void suspend_task_set(void){
//	uint32_t i;

	uint32_t allTasks_suspended = 0;

	while(allTasks_suspended != TASKS_NUMBER){
		taskENTER_CRITICAL();
		if(delayed[0]){
			osThreadSuspend(t1TaskHandle);
			delayed[0] = 0;
			allTasks_suspended++;
		}
		if(delayed[1]){
			osThreadSuspend(t2TaskHandle);
			delayed[1] = 0;
			allTasks_suspended++;
		}
		if(delayed[2]){
			osThreadSuspend(t3TaskHandle);
			delayed[2] = 0;
			allTasks_suspended++;
		}
		if(delayed[3]){
			osThreadSuspend(t4TaskHandle);
			delayed[3] = 0;
			allTasks_suspended++;
		}
		if(delayed[4]){
			osThreadSuspend(t5TaskHandle);
			delayed[4] = 0;
			allTasks_suspended++;
		}
		if(delayed[5]){
			osThreadSuspend(t6TaskHandle);
			delayed[5] = 0;
			allTasks_suspended++;
		}
		if(delayed[6]){
			osThreadSuspend(t7TaskHandle);
			delayed[6] = 0;
			allTasks_suspended++;
		}
		if(delayed[7]){
			osThreadSuspend(t8TaskHandle);
			delayed[7] = 0;
			allTasks_suspended++;
		}
		if(delayed[8]){
			osThreadSuspend(t9TaskHandle);
			delayed[8] = 0;
			allTasks_suspended++;
		}
		if(delayed[9]){
			osThreadSuspend(t10TaskHandle);
			delayed[9] = 0;
			allTasks_suspended++;
		}
		if(delayed[10]){
			osThreadSuspend(t11TaskHandle);
			delayed[10] = 0;
			allTasks_suspended++;
		}
		if(delayed[11]){
			osThreadSuspend(t12TaskHandle);
			delayed[11] = 0;
			allTasks_suspended++;
		}
		if(delayed[12]){
			osThreadSuspend(t13TaskHandle);
			delayed[12] = 0;
			allTasks_suspended++;
		}
		if(delayed[13]){
			osThreadSuspend(t14TaskHandle);
			delayed[13] = 0;
			allTasks_suspended++;
		}
		if(delayed[14]){
			osThreadSuspend(t15TaskHandle);
			delayed[14] = 0;
			allTasks_suspended++;
		}
		if(delayed[15]){
			osThreadSuspend(t16TaskHandle);
			delayed[15] = 0;
			allTasks_suspended++;
		}
		taskEXIT_CRITICAL();
		osDelay(40);
	}
}

void adapt_Sch_Test_Pin_Callback(void){
	if(mainTaskHandle != NULL){
		osThreadResume(mainTaskHandle);
		disable_adapt_duty_cycle();
	}
}
//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
//	received_spi = 1;
//}

//void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
//	received_spi = 1;
//}

void init_delays(void){
	uint16_t i;
	for(i=0; i<TASKS_NUMBER; i++){
		avg_delay[i] = 0;
		num_counts[i] = 0;
		min_delay[i] = 4000000000;
		max_delay[i] =	0;
	}
}

//uint32_t data_to_send[(TASKS_NUMBER*4)*sizeof(uint32_t)];
void send_data(void){
	uint16_t i;
	uint32_t* data_to_send;
	uint32_t* data_rcv;
	data_to_send = (uint32_t*) pvPortMalloc((TASKS_NUMBER*4)*sizeof(uint32_t));
	data_rcv  = (uint32_t*) pvPortMalloc((TASKS_NUMBER*4)*sizeof(uint32_t));
	taskENTER_CRITICAL();

	for(i=0; i < TASKS_NUMBER; i++){
		if(num_counts[i] > 0){
			avg_delay[i] = avg_delay[i]/num_counts[i];
		}
		else{
			max_delay[i] = 3000000000;
			avg_delay[i] = 2000000000;
		}
		data_to_send[(i*4)] = avg_delay[i];
		data_to_send[(i*4)+1] = num_counts[i];
		data_to_send[(i*4)+2] = max_delay[i];
		data_to_send[(i*4)+3] = min_delay[i];
	}


	taskEXIT_CRITICAL();

	port_state_t spi_state = spi1_driver->spi_funcs.spi_port_get_state(spi1_driver->spi_vars, spi_test_id);
	if(spi_state != DRIVER_PORT_CLOSED){//RCV	COMMAND
		spi1_driver->spi_funcs.spi_read_write_nb(spi1_driver->spi_vars, spi_test_id,(uint8_t *) data_to_send, (uint8_t *) data_rcv, (TASKS_NUMBER*4)*sizeof(uint32_t));
	}
//	HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t*) data_to_send, (uint8_t*) data_rcv, (TASKS_NUMBER*4)*sizeof(uint32_t));	//Transmit data
//	while(!received_spi){
//
//	}
//	received_spi = 0;
	vPortFree(data_to_send);
	vPortFree(data_rcv);
}
