/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include <stdio.h>
#include "lptim.h"
#include "leds.h"
#include "stm32l4xx_hal.h"
#include "main_process.h"
#include "vcom_usb_inout.h"
#include "usb_device.h"
#include "usbd_desc.h"
#include "time_control.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId InitTaskHandle;

/* USER CODE BEGIN Variables */
//#define LPTIM_CLOCK_HZ					32768 //NODOS
#define LPTIM_CLOCK_HZ					32000    //TRANSMISOR
#define TICKS_PER_SYSTICK_LOW_POWER		(LPTIM_CLOCK_HZ/configTICK_RATE_HZ)
#define MAX_LOW_POWER_COUNT 			0xFFFF
#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )

extern time_control_t* time_control;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void InitTask_main(void const * argument);

extern void MX_FATFS_Init(void);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Pre/Post sleep processing prototypes */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime);
void PostSleepProcessing(uint32_t *ulExpectedIdleTime);

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	leds_on(LEDS_RED2);
	while(1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	leds_on(LEDS_RED2);
	while(1);
}
/* USER CODE END 5 */

/* USER CODE BEGIN PREPOSTSLEEP */
void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
	uint32_t lowpower_ticks =((*ulExpectedIdleTime)*TICKS_PER_SYSTICK_LOW_POWER)-2;		//-2: Segun el datasheet el timer tarda dos ciclos en iniciarse

	if(lowpower_ticks >= MAX_LOW_POWER_COUNT){
		lowpower_ticks = MAX_LOW_POWER_COUNT-1;
	}

	HAL_SuspendTick();

	__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);		//Limpio el flag
	HAL_LPTIM_TimeOut_Start_IT(&hlptim1, MAX_LOW_POWER_COUNT, lowpower_ticks);

	portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
	*ulExpectedIdleTime = 0;
}

void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
/* place for user code */
	uint32_t count = HAL_LPTIM_ReadCounter(&hlptim1);
	HAL_LPTIM_TimeOut_Stop_IT(&hlptim1);
	__HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPM);		//Limpio el flag de interrupcion. No hace falta que salte, ya se ha despertado el micro

	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}

	//	uwTick += ((count)*1000)/ LPTIM_CLOCK_HZ;		//Update HAL time in ms			//Comentarlo en caso de que lo que se quiera no sea mantener los tiempos sino detener la hebra un determinado tiempo con HAL_Delay al que se le sumaria el tiempo dormido
		HAL_ResumeTick();

	(*ulExpectedIdleTime) = ((count)*configTICK_RATE_HZ)/ LPTIM_CLOCK_HZ;			//How many time has actually lapsed
}
/* USER CODE END PREPOSTSLEEP */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of InitTask */
  osThreadDef(InitTask, InitTask_main, osPriorityNormal, 0, 256);
  InitTaskHandle = osThreadCreate(osThread(InitTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
#if DEBUG_USB
  /* USER CODE END RTOS_QUEUES */
}

/* InitTask_main function */
void InitTask_main(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN InitTask_main */

	osDelay(3500);
#else
}
void InitTask_main(void const * argument)
{
	MX_FATFS_Init();
#endif
//  HAL_DBGMCU_DisableDBGStsopMode();
    HAL_RCCEx_WakeUpStopCLKConfig(RCC_STOP_WAKEUPCLOCK_MSI);		//SET MSI clock when returning from sleep
	main_process();
	/*Call main function of the system*/

	/*This infinite loop is never reached*/
  /* Infinite loop */
  for(;;)
  {
	osDelay(1);
  }
  /* USER CODE END InitTask_main */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
