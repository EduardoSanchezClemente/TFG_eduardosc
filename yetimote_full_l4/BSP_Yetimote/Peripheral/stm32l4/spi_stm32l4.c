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
 * spi_stm32l4.c
 *
 *  Created on: 16 de ene. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file 	spi_stm32l4.c
 * @brief	Driver for STM32L4 SPI. This driver uses always the DMA for SPI communications when the read/write operation is non blocking
 */

#include "spi.h"
#include "os_spi.h"


extern uint8_t os_periph_dma_active;

osSemaphoreDef (spi2_semaphore);    // Declare semaphore
osSemaphoreId  spi2_semaphore_id = NULL; 		// Semaphore ID
osSemaphoreDef (spi1_semaphore);    // Declare semaphore
osSemaphoreId  spi1_semaphore_id = NULL; 		// Semaphore ID

spi_funcs_t stm32l4_spi_funcs;
#if FULL_DEVICE_DRIVER
spi_config_funcs_t stm32l4_spi_config_funcs;
#endif
/* SPI FUNCTIONS FOR STM32L4*/
retval_t stm32l4_spi_init(spi_driver_t* spi_driver, spi_mode_t	spi_mode, spi_lines_t spi_lines, uint8_t data_size, spi_polarity_t spi_polarity,
		spi_phase_t spi_phase, spi_nss_mode_t spi_nss, uint32_t spi_speed, uint32_t default_spi_timeout);
retval_t stm32l4_spi_deinit(spi_driver_t* spi_driver);
int16_t stm32l4_spi_open(spi_driver_t* spi_driver, uint8_t enable_sw_cs, uint16_t cs_hw_pin, uint16_t cs_hw_port);
retval_t stm32l4_spi_close(spi_driver_t* spi_driver, int16_t port_number);
retval_t stm32l4_spi_read(spi_driver_t* spi_driver, int16_t port_number, uint8_t *prx, uint16_t size);
retval_t stm32l4_spi_read_nb(spi_driver_t* spi_driver, int16_t port_number, uint8_t *prx, uint16_t size);
retval_t stm32l4_spi_write(spi_driver_t* spi_driver, int16_t port_number, uint8_t *ptx, uint16_t size);
retval_t stm32l4_spi_write_nb(spi_driver_t* spi_driver, int16_t port_number, uint8_t *ptx, uint16_t size);
retval_t stm32l4_spi_read_write(spi_driver_t* spi_driver, int16_t port_number, uint8_t *ptx, uint8_t *prx, uint16_t size);
retval_t stm32l4_spi_read_write_nb(spi_driver_t* spi_driver, int16_t port_number, uint8_t *ptx, uint8_t *prx, uint16_t size);
port_state_t stm32l4_spi_port_get_state(spi_driver_t* spi_driver, int16_t port_number);

/* SPI CONFIG FUNCTIONS FOR STM32L4*/
#if SPI_DRIVER_CONFIG_FUNCS
retval_t stm32l4_spi_set_mode(spi_driver_t* spi_driver, spi_mode_t	spi_mode);
retval_t stm32l4_spi_set_lines(spi_driver_t* spi_driver, spi_lines_t spi_lines);
retval_t stm32l4_spi_set_data_size(spi_driver_t* spi_driver, uint8_t data_size);
retval_t stm32l4_spi_set_polarity(spi_driver_t* spi_driver, spi_polarity_t spi_polarity);
retval_t stm32l4_spi_set_phase(spi_driver_t* spi_driver, spi_phase_t spi_phase);
retval_t stm32l4_spi_set_nss_mode(spi_driver_t* spi_driver, spi_nss_mode_t spi_nss);
retval_t stm32l4_spi_set_speed(spi_driver_t* spi_driver, uint32_t spi_speed);
retval_t stm32l4_spi_set_timeout(spi_driver_t* spi_driver, uint32_t default_spi_timeout);
spi_mode_t stm32l4_spi_get_mode(spi_driver_t* spi_driver);
spi_lines_t stm32l4_spi_get_lines(spi_driver_t* spi_driver);
uint8_t stm32l4_spi_get_data_size(spi_driver_t* spi_driver);
spi_polarity_t stm32l4_spi_get_polarity(spi_driver_t* spi_driver);
spi_phase_t stm32l4_spi_get_phase(spi_driver_t* spi_driver);
spi_nss_mode_t stm32l4_spi_get_nss_mode(spi_driver_t* spi_driver);
uint32_t stm32l4_spi_get_speed(spi_driver_t* spi_driver);
uint32_t stm32l4_spi_get_timeout(spi_driver_t* spi_driver);
spi_driver_state_t stm32l4_spi_driver_get_state (spi_driver_t* spi_driver);

spi_config_funcs_t stm32l4_spi_config_funcs = {
		stm32l4_spi_set_mode,
		stm32l4_spi_set_lines,
		stm32l4_spi_set_data_size,
		stm32l4_spi_set_polarity,
		stm32l4_spi_set_phase,
		stm32l4_spi_set_nss_mode,
		stm32l4_spi_set_speed,
		stm32l4_spi_set_timeout,
		stm32l4_spi_get_mode,
		stm32l4_spi_get_lines,
		stm32l4_spi_get_data_size,
		stm32l4_spi_get_polarity,
		stm32l4_spi_get_phase,
		stm32l4_spi_get_nss_mode,
		stm32l4_spi_get_speed,
		stm32l4_spi_get_timeout,
		stm32l4_spi_driver_get_state,
};
#endif

spi_funcs_t stm32l4_spi_funcs = {
		stm32l4_spi_init,
		stm32l4_spi_deinit,
		stm32l4_spi_open,
		stm32l4_spi_close,
		stm32l4_spi_read,
		stm32l4_spi_read_nb,
		stm32l4_spi_write,
		stm32l4_spi_write_nb,
		stm32l4_spi_read_write,
		stm32l4_spi_read_write_nb,
		stm32l4_spi_port_get_state,
};

/**
 *
 * @param spi_driver->spi_vars
 * @param spi_mode
 * @param spi_lines
 * @param data_size
 * @param spi_polarity
 * @param spi_phase
 * @param spi_nss
 * @param spi_speed
 * @param default_spi_timeout
 * @return
 */
retval_t stm32l4_spi_init(spi_driver_t* spi_driver, spi_mode_t	spi_mode, spi_lines_t spi_lines, uint8_t data_size, spi_polarity_t spi_polarity,
		spi_phase_t spi_phase, spi_nss_mode_t spi_nss, uint32_t spi_speed, uint32_t default_spi_timeout){

	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}

	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);

	if(spi_driver->spi_vars->driver_state != DRIVER_NOT_INIT){	//Return error if the driver has been previously initiated
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}


	SPI_InitTypeDef* tempSpiInit = (SPI_InitTypeDef*) pvPortMalloc(sizeof(SPI_InitTypeDef));


	spi_driver->spi_vars->spi_mode = spi_mode;
	switch(spi_mode){							//SET SPI MODE
	case SPI_MASTER:
		tempSpiInit->Mode = SPI_MODE_MASTER;
		break;
	case SPI_SLAVE:
		tempSpiInit->Mode = SPI_MODE_SLAVE;
		break;
	default:
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		vPortFree(tempSpiInit);
		return RET_ERROR;
		break;
	}

	spi_driver->spi_vars->spi_lines = spi_lines;
	switch(spi_lines){							//SET LINES DIRECTION
	case LINES_RX:
		tempSpiInit->Direction = SPI_DIRECTION_2LINES_RXONLY;
		break;
	case LINES_DUPLEX:
		tempSpiInit->Direction = SPI_DIRECTION_2LINES;
		break;
	default:
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		vPortFree(tempSpiInit);
		return RET_ERROR;
		break;
	}

	if((data_size < 4) || (data_size > 16)){
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		vPortFree(tempSpiInit);
		return RET_ERROR;
	}
	spi_driver->spi_vars->data_size = data_size;				//SET DATA SIZE
	uint32_t set_data_size = (uint32_t) data_size - 4;
	set_data_size = (set_data_size << 8);
	set_data_size += SPI_DATASIZE_4BIT;
	tempSpiInit->DataSize = set_data_size;

	spi_driver->spi_vars->spi_polarity = spi_polarity;
	switch(spi_polarity){								//SET SPI POLARITY
	case SPI_POL_HIGH:
		tempSpiInit->CLKPolarity = SPI_POLARITY_HIGH;
		break;
	case SPI_POL_LOW:
		tempSpiInit->CLKPolarity = SPI_POLARITY_LOW;
		break;
	default:
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		vPortFree(tempSpiInit);
		return RET_ERROR;
		break;
	}

	spi_driver->spi_vars->spi_phase = spi_phase;				//SET SPI PHASE
	switch(spi_phase){
	case SPI_1EDGE:
		tempSpiInit->CLKPhase = SPI_PHASE_1EDGE;
		break;
	case SPI_2EDGE:
		tempSpiInit->CLKPhase = SPI_PHASE_2EDGE;
		break;
	default:
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		vPortFree(tempSpiInit);
		return RET_ERROR;
		break;
	}

	spi_driver->spi_vars->spi_nss = spi_nss;					//SET SPI NSS SIGNAL
	switch(spi_nss){
	case SPI_SOFT_NSS:
		tempSpiInit->NSS = SPI_NSS_SOFT;
		break;
	case SPI_HW_NSS_INPUT:
		tempSpiInit->NSS = SPI_NSS_HARD_INPUT;
		break;
	case SPI_HW_NSS_OUTPUT:
		tempSpiInit->NSS = SPI_NSS_HARD_OUTPUT;
		break;
	default:
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		vPortFree(tempSpiInit);
		return RET_ERROR;
		break;
	}

	uint32_t prescaler = (uint32_t) HAL_RCC_GetHCLKFreq()/spi_speed;	//SET SPI SPEED. IT IS DONE WITH THE BAUDRATE PRESCALER

	if(prescaler > 128){												//SPI BAUDRATE IS CALCULATED TO SET SPEED TO THE NEAREST LOWER VALUE
		spi_driver->spi_vars->spi_speed = (uint32_t) HAL_RCC_GetHCLKFreq()/256;
		tempSpiInit->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	}
	else if(prescaler > 64){
		spi_driver->spi_vars->spi_speed = (uint32_t) HAL_RCC_GetHCLKFreq()/128;
		tempSpiInit->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	}
	else if(prescaler > 32){
		spi_driver->spi_vars->spi_speed = (uint32_t) HAL_RCC_GetHCLKFreq()/64;
		tempSpiInit->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	}
	else if(prescaler > 16){
		spi_driver->spi_vars->spi_speed = (uint32_t) HAL_RCC_GetHCLKFreq()/32;
		tempSpiInit->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	}
	else if(prescaler > 8){
		spi_driver->spi_vars->spi_speed = (uint32_t) HAL_RCC_GetHCLKFreq()/16;
		tempSpiInit->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	}
	else if(prescaler > 4){
		spi_driver->spi_vars->spi_speed = (uint32_t) HAL_RCC_GetHCLKFreq()/8;
		tempSpiInit->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	}
	else if(prescaler > 2){
		spi_driver->spi_vars->spi_speed = (uint32_t) HAL_RCC_GetHCLKFreq()/4;
		tempSpiInit->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	}
	else if(prescaler >= 0){
		spi_driver->spi_vars->spi_speed = (uint32_t) HAL_RCC_GetHCLKFreq()/2;
		tempSpiInit->BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	}
	else{
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		vPortFree(tempSpiInit);
		return RET_ERROR;
	}

	if(default_spi_timeout == 0){									//Set SPI bus default timeout
		spi_driver->spi_vars->default_spi_timeout = osWaitForever;
	}
	else{
		spi_driver->spi_vars->default_spi_timeout = default_spi_timeout;
	}

	switch(spi_driver->spi_vars->spi_hw_number){	//The hardware spi device used. For the yetimote STM32L4 SPI2 is used for transceiver control. If another SPI is used, just add a case with the number.
	case 1:
		hspi1.Instance = SPI1;
		hspi1.Init.Mode = tempSpiInit->Mode;
		hspi1.Init.Direction = tempSpiInit->Direction;
		hspi1.Init.DataSize = tempSpiInit->DataSize;
		hspi1.Init.CLKPolarity = tempSpiInit->CLKPolarity;
		hspi1.Init.CLKPhase = tempSpiInit->CLKPhase;
		hspi1.Init.NSS = tempSpiInit->NSS;
		hspi1.Init.BaudRatePrescaler = tempSpiInit->BaudRatePrescaler;
		hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
		hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi1.Init.CRCPolynomial = 7;
		hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
		vPortFree(tempSpiInit);
		if (HAL_SPI_Init(&hspi1) != HAL_OK)
		{
			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_ERROR;
		}
		if(spi1_semaphore_id == NULL){
			spi1_semaphore_id = osSemaphoreCreate(osSemaphore(spi1_semaphore), 1);
			osSemaphoreWait(spi1_semaphore_id, osWaitForever);
		}
		break;
	case 2:
		hspi2.Instance = SPI2;
		hspi2.Init.Mode = tempSpiInit->Mode;
		hspi2.Init.Direction = tempSpiInit->Direction;
		hspi2.Init.DataSize = tempSpiInit->DataSize;
		hspi2.Init.CLKPolarity = tempSpiInit->CLKPolarity;
		hspi2.Init.CLKPhase = tempSpiInit->CLKPhase;
		hspi2.Init.NSS = tempSpiInit->NSS;
		hspi2.Init.BaudRatePrescaler = tempSpiInit->BaudRatePrescaler;
		hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
		hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
		hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		hspi2.Init.CRCPolynomial = 7;
		hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
		vPortFree(tempSpiInit);
		if (HAL_SPI_Init(&hspi2) != HAL_OK)
		{
			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_ERROR;
		}
		if(spi2_semaphore_id == NULL){
			spi2_semaphore_id = osSemaphoreCreate(osSemaphore(spi2_semaphore), 1);
			osSemaphoreWait(spi2_semaphore_id, osWaitForever);
		}
		break;

	default:
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		vPortFree(tempSpiInit);
		return RET_ERROR;	//Wrong hardware spi number
		break;
	}

	if(spi_driver->spi_vars->opened_ports == NULL){
		spi_driver->spi_vars->opened_ports = gen_list_init();		//Create list for the opened ports structures
	}
	spi_driver->spi_vars->driver_state = DRIVER_READY;

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spi_driver->spi_vars
 * @return
 */
retval_t stm32l4_spi_deinit(spi_driver_t* spi_driver){

	if(spi_driver == NULL){		//return error if the driver variables does not exist
			return RET_ERROR;
		}

	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);

	if(spi_driver->spi_vars->driver_state != DRIVER_READY){	//Return error if the driver has not been initiated or it is busy
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}

	switch(spi_driver->spi_vars->spi_hw_number){
	case 1:
		if (HAL_SPI_DeInit(&hspi1) != HAL_OK)
		{
			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_ERROR;
		}
		osSemaphoreRelease(spi1_semaphore_id);		//Make sure we release the semaphore before destroying
		osSemaphoreRelease(spi1_semaphore_id);
		osSemaphoreDelete(spi1_semaphore_id);
		spi1_semaphore_id = NULL;
		break;

	case 2:
		if (HAL_SPI_DeInit(&hspi2) != HAL_OK)
		{
			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_ERROR;
		}
		osSemaphoreRelease(spi2_semaphore_id);		//Make sure we release the semaphore before destroying
		osSemaphoreRelease(spi2_semaphore_id);
		osSemaphoreDelete(spi2_semaphore_id);
		spi2_semaphore_id = NULL;
		break;
	default:
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
		break;
	}

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;

	while(current->next != NULL){				//Close all opened ports for de-initializing

		temp_list_spi_port = (spi_port_t*) current->next->item;
		while(temp_list_spi_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to close it(end all transmissions)

		if(temp_list_spi_port->sw_cs > 0){
			GPIO_TypeDef* GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
			uint32_t GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
			HAL_GPIO_DeInit(GPIO_Port, GPIO_Pin);
		}

		vPortFree(temp_list_spi_port);
		gen_list* next = current->next->next;
		vPortFree(current->next);
		current->next = next;
	}

	gen_list_remove_all(spi_driver->spi_vars->opened_ports);
	spi_driver->spi_vars->opened_ports = NULL;

	spi_driver->spi_vars->driver_state = DRIVER_NOT_INIT;
	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
	return RET_OK;
}

/**
 *
 * @param spi_driver->spi_vars
 * @param enable_sw_cs
 * @param cs_hw_pin
 * @param cs_hw_port
 * @return
 */
int16_t stm32l4_spi_open(spi_driver_t* spi_driver, uint8_t enable_sw_cs, uint16_t cs_hw_pin, uint16_t cs_hw_port){
	if(spi_driver == NULL){		//return error if the driver variables does not exist
			return -1;
		}

	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);

	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return -1;
	}

	spi_port_t* new_spi_port = (spi_port_t*) pvPortMalloc(sizeof(spi_port_t));

	new_spi_port->sw_cs = enable_sw_cs;

	if(enable_sw_cs > 0){
		if((cs_hw_port>8) || (cs_hw_pin>15)){	//Return error if the port number or pin is out of limits (GPIO_PORT_A-H) (GPIO_PIN_0-15)
			vPortFree(new_spi_port);
			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return -1;
		}
		new_spi_port->cs_hw_pin = cs_hw_pin;
		new_spi_port->cs_hw_port = cs_hw_port;

		switch(cs_hw_port){
		case 0:
			__HAL_RCC_GPIOA_CLK_ENABLE();
			break;
		case 1:
			__HAL_RCC_GPIOB_CLK_ENABLE();
			break;
		case 2:
			__HAL_RCC_GPIOC_CLK_ENABLE();
			break;
		case 3:
			__HAL_RCC_GPIOD_CLK_ENABLE();
			break;
		case 4:
			__HAL_RCC_GPIOE_CLK_ENABLE();
			break;
		case 5:
			__HAL_RCC_GPIOF_CLK_ENABLE();
			break;
		case 6:
			__HAL_RCC_GPIOG_CLK_ENABLE();
			break;
		case 7:
			__HAL_RCC_GPIOH_CLK_ENABLE();
			break;
		default:
			vPortFree(new_spi_port);
			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return -1;
			break;
		}

		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_TypeDef* GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((cs_hw_port*4)<<8));
		uint32_t GPIO_Pin = (0x0001<<cs_hw_pin);
		HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);

		GPIO_InitStruct.Pin = GPIO_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		HAL_GPIO_Init(GPIO_Port, &GPIO_InitStruct);
	}

	new_spi_port->port_id = 0;								//Check the opened port list to select an ordered available port_id

	spi_port_t* temp_list_spi_port;
	gen_list* current = spi_driver->spi_vars->opened_ports;
	uint8_t repeat_loop = 1;
	while(repeat_loop){
		repeat_loop = 0;
		current = spi_driver->spi_vars->opened_ports;
		while(current->next != NULL){
			temp_list_spi_port = (spi_port_t*) current->next->item;
			if(temp_list_spi_port->port_id == new_spi_port->port_id){
				repeat_loop = 1;
				new_spi_port->port_id++;
			}
			current = current->next;
		}
	}

	new_spi_port->port_state = DRIVER_PORT_OPENED_READY;

	gen_list_add(spi_driver->spi_vars->opened_ports, (void*) new_spi_port);
	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
	return new_spi_port->port_id;
}

/**
 *
 * @param spi_driver->spi_vars
 * @param port_number
 * @return
 */
retval_t stm32l4_spi_close(spi_driver_t* spi_driver, int16_t port_number){
	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}

	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);

	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;

	while(current->next != NULL){

		temp_list_spi_port = (spi_port_t*) current->next->item;
		if(temp_list_spi_port->port_id == port_number){

			while(temp_list_spi_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to close it(end all transmissions)

			if(temp_list_spi_port->sw_cs > 0){
				GPIO_TypeDef* GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
				uint32_t GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
				HAL_GPIO_DeInit(GPIO_Port, GPIO_Pin);
			}
			gen_list* next = current->next->next;
			temp_list_spi_port->port_state = DRIVER_PORT_CLOSED;
			vPortFree(temp_list_spi_port);
			vPortFree(current->next);
			current->next = next;

			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_OK;										//Port succesfully deleted
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
	return RET_ERROR;										//Port not found. Not deleted
}

/**
 *
 * @param spi_driver->spi_vars
 * @param port_number
 * @param prx
 * @param size
 * @return
 */
retval_t stm32l4_spi_read(spi_driver_t* spi_driver, int16_t port_number, uint8_t *prx, uint16_t size){
	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);
	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}

	while(spi_driver->spi_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;
	while(current->next != NULL){

		temp_list_spi_port = (spi_port_t*) current->next->item;
		if(temp_list_spi_port->port_id == port_number){

			while(temp_list_spi_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)
			uint8_t* nullTx = (uint8_t*) pvPortMalloc(size);

			GPIO_TypeDef* GPIO_Port;
			uint32_t GPIO_Pin;

			switch(spi_driver->spi_vars->spi_hw_number){
			case 1:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive(&hspi1, nullTx, prx, size, spi_driver->spi_vars->default_spi_timeout);		//RECEIVE DATA

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				vPortFree(nullTx);
				break;
			case 2:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive(&hspi2, nullTx, prx, size, spi_driver->spi_vars->default_spi_timeout);		//RECEIVE DATA

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				vPortFree(nullTx);
				break;
			default:
				vPortFree(nullTx);
				osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param spi_driver->spi_vars
 * @param port_number
 * @param prx
 * @param size
 * @return
 */
retval_t stm32l4_spi_read_nb(spi_driver_t* spi_driver, int16_t port_number, uint8_t *prx, uint16_t size){
	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);
	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}

	while(spi_driver->spi_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;
	while(current->next != NULL){

		temp_list_spi_port = (spi_port_t*) current->next->item;
		if(temp_list_spi_port->port_id == port_number){

			while(temp_list_spi_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			uint8_t* nullTx = (uint8_t*) pvPortMalloc(size);

			GPIO_TypeDef* GPIO_Port;
			uint32_t GPIO_Pin;
			switch(spi_driver->spi_vars->spi_hw_number){
			case 1:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive_DMA(&hspi1, nullTx, prx, size);		//RECEIVE DATA
				osSemaphoreWait(spi1_semaphore_id, spi_driver->spi_vars->default_spi_timeout);

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				vPortFree(nullTx);
				break;
			case 2:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive_DMA(&hspi2, nullTx, prx, size);		//RECEIVE DATA
				osSemaphoreWait(spi2_semaphore_id, spi_driver->spi_vars->default_spi_timeout);

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				vPortFree(nullTx);
				break;
			default:
				vPortFree(nullTx);
				osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param spi_driver->spi_vars
 * @param port_number
 * @param ptx
 * @param size
 * @return
 */
retval_t stm32l4_spi_write(spi_driver_t* spi_driver, int16_t port_number, uint8_t *ptx, uint16_t size){
	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);
	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}

	while(spi_driver->spi_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;
	while(current->next != NULL){

		temp_list_spi_port = (spi_port_t*) current->next->item;
		if(temp_list_spi_port->port_id == port_number){

			while(temp_list_spi_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			uint8_t* nullRx = (uint8_t*) pvPortMalloc(size);

			GPIO_TypeDef* GPIO_Port;
			uint32_t GPIO_Pin;
			switch(spi_driver->spi_vars->spi_hw_number){
			case 1:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive(&hspi1, ptx, nullRx, size, spi_driver->spi_vars->default_spi_timeout);		//SEND DATA

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				vPortFree(nullRx);
				break;
			case 2:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive(&hspi2, ptx, nullRx, size, spi_driver->spi_vars->default_spi_timeout);		//SEND DATA

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				vPortFree(nullRx);
				break;
			default:
				vPortFree(nullRx);
				osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param spi_driver->spi_vars
 * @param port_number
 * @param ptx
 * @param size
 * @return
 */
retval_t stm32l4_spi_write_nb(spi_driver_t* spi_driver, int16_t port_number, uint8_t *ptx, uint16_t size){
	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);
	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}

	while(spi_driver->spi_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;
	while(current->next != NULL){

		temp_list_spi_port = (spi_port_t*) current->next->item;
		if(temp_list_spi_port->port_id == port_number){

			while(temp_list_spi_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			uint8_t* nullRx = (uint8_t*) pvPortMalloc(size);

			GPIO_TypeDef* GPIO_Port;
			uint32_t GPIO_Pin;
			switch(spi_driver->spi_vars->spi_hw_number){
			case 1:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive_DMA(&hspi1, ptx, nullRx, size);					//SEND DATA
				osSemaphoreWait(spi1_semaphore_id, spi_driver->spi_vars->default_spi_timeout);

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				vPortFree(nullRx);
				break;
			case 2:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive_DMA(&hspi2, ptx, nullRx, size);					//SEND DATA
				osSemaphoreWait(spi2_semaphore_id, spi_driver->spi_vars->default_spi_timeout);

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				vPortFree(nullRx);
				break;
			default:
				vPortFree(nullRx);
				osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param spi_driver->spi_vars
 * @param port_number
 * @param ptx
 * @param prx
 * @param size
 * @return
 */
retval_t stm32l4_spi_read_write(spi_driver_t* spi_driver, int16_t port_number, uint8_t *ptx, uint8_t *prx, uint16_t size){
	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);
	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}

	while(spi_driver->spi_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;
	while(current->next != NULL){

		temp_list_spi_port = (spi_port_t*) current->next->item;
		if(temp_list_spi_port->port_id == port_number){

			while(temp_list_spi_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			GPIO_TypeDef* GPIO_Port;
			uint32_t GPIO_Pin;
			switch(spi_driver->spi_vars->spi_hw_number){
			case 1:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive(&hspi1, ptx, prx, size, spi_driver->spi_vars->default_spi_timeout);		//SEND RECEIVE DATA

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				break;
			case 2:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive(&hspi2, ptx, prx, size, spi_driver->spi_vars->default_spi_timeout);		//SEND RECEIVE DATA

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
				break;
			default:
				osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param spi_driver->spi_vars
 * @param port_number
 * @param ptx
 * @param prx
 * @param size
 * @return
 */
retval_t stm32l4_spi_read_write_nb(spi_driver_t* spi_driver, int16_t port_number, uint8_t *ptx, uint8_t *prx, uint16_t size){
	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);
	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return RET_ERROR;
	}

	while(spi_driver->spi_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;
	while(current->next != NULL){

		temp_list_spi_port = (spi_port_t*) current->next->item;
		if(temp_list_spi_port->port_id == port_number){

			while(temp_list_spi_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			GPIO_TypeDef* GPIO_Port;
			uint32_t GPIO_Pin;
			switch(spi_driver->spi_vars->spi_hw_number){
			case 1:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive_DMA(&hspi1, ptx, prx, size);					//SEND RECEIVE DATA
				osSemaphoreWait(spi1_semaphore_id, spi_driver->spi_vars->default_spi_timeout);

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
					break;
			case 2:
				if(temp_list_spi_port->sw_cs > 0){		//ACTIVE CHIP SELECT
					GPIO_Port = (GPIO_TypeDef*) (GPIOA_BASE + ((temp_list_spi_port->cs_hw_port*4)<<8));	//De init the cs
					GPIO_Pin = (0x0001<<temp_list_spi_port->cs_hw_pin);
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
				}
				spi_driver->spi_vars->driver_state = DRIVER_BUSY;
				temp_list_spi_port->port_state = DRIVER_PORT_READING;
//				taskENTER_CRITICAL();
				os_periph_dma_active = 1;
//				taskEXIT_CRITICAL();

				HAL_SPI_TransmitReceive_DMA(&hspi2, ptx, prx, size);					//SEND RECEIVE DATA
				osSemaphoreWait(spi2_semaphore_id, spi_driver->spi_vars->default_spi_timeout);

				os_periph_dma_active = 0;
				spi_driver->spi_vars->driver_state = DRIVER_READY;
				temp_list_spi_port->port_state = DRIVER_PORT_OPENED_READY;
				if(temp_list_spi_port->sw_cs > 0){		//DISABLE CHIP SELECT
					HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
				}
					break;
			default:
				osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param spi_driver->spi_vars
 * @param port_number
 * @return
 */
port_state_t stm32l4_spi_port_get_state(spi_driver_t* spi_driver, int16_t port_number){

	if(spi_driver == NULL){		//return error if the driver variables does not exist
		return DRIVER_PORT_CLOSED;
	}
	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);
	if(spi_driver->spi_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(spi_driver->spi_vars->spi_mutex_id);
		return DRIVER_PORT_CLOSED;
	}

	gen_list* current = spi_driver->spi_vars->opened_ports;
	spi_port_t* temp_list_spi_port;
	while(current->next != NULL){

		temp_list_spi_port = (spi_port_t*) current->next->item;
		if(temp_list_spi_port->port_id == port_number){

			port_state_t port_state = temp_list_spi_port->port_state;
			osMutexRelease(spi_driver->spi_vars->spi_mutex_id);

			return port_state;										//return port state
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);						//Port id not found
	return DRIVER_PORT_CLOSED;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance == SPI1){
		osSemaphoreRelease(spi1_semaphore_id);
	}
	if(hspi->Instance == SPI2){
		osSemaphoreRelease(spi2_semaphore_id);
	}
}
