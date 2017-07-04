/*
 * Copyright (c) 2017, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * i2c_stm32l4.c
 *
 *  Created on: 6 de feb. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file i2c_stm32l4.c
 */

#include "i2c.h"
#include "os_i2c.h"


extern uint8_t os_periph_dma_active;

osSemaphoreDef (i2c1_semaphore);    // Declare semaphore
osSemaphoreId  i2c1_semaphore_id = NULL; 		// Semaphore ID

i2c_funcs_t stm32l4_i2c_funcs;


// CUIDADO. LAS FUNCIONES NON BLOCKING USANDO DMA EN EL BUS I2C NO TERMINAN DE FUNCIONAR. SE CUELGAN A VECES

/* SPI FUNCTIONS FOR STM32L4*/
retval_t stm32l4_i2c_init(i2c_driver_t* i2c_driver, uint8_t own_addr, uint32_t default_i2c_timeout);
retval_t stm32l4_i2c_deinit(i2c_driver_t* i2c_driver);
int16_t stm32l4_i2c_open(i2c_driver_t* i2c_driver, uint8_t device_addr, i2c_port_mode_t i2c_port_mode);
retval_t stm32l4_i2c_close(i2c_driver_t* i2c_driver, int16_t port_number);
retval_t stm32l4_i2c_read(i2c_driver_t* i2c_driver, int16_t port_number, uint8_t *prx, uint16_t size);
retval_t stm32l4_i2c_read_nb(i2c_driver_t* i2c_driver, int16_t port_number, uint8_t *prx, uint16_t size);
retval_t stm32l4_i2c_write(i2c_driver_t* i2c_driver, int16_t port_number, uint8_t *ptx, uint16_t size);
retval_t stm32l4_i2c_write_nb(i2c_driver_t* i2c_driver, int16_t port_number, uint8_t *ptx, uint16_t size);
retval_t stm32l4_i2c_read_reg(i2c_driver_t* i2c_driver, int16_t port_number, uint16_t reg_addr, uint16_t reg_addr_size, uint8_t *prx, uint16_t size);
retval_t stm32l4_i2c_write_reg(i2c_driver_t* i2c_driver, int16_t port_number, uint16_t reg_addr, uint16_t reg_addr_size, uint8_t *ptx, uint16_t size);
port_state_t stm32l4_i2c_port_get_state(i2c_driver_t* i2c_driver, int16_t port_number);

i2c_funcs_t stm32l4_i2c_funcs = {
		stm32l4_i2c_init,
		stm32l4_i2c_deinit,
		stm32l4_i2c_open,
		stm32l4_i2c_close,
		stm32l4_i2c_read,
		stm32l4_i2c_read_nb,
		stm32l4_i2c_write,
		stm32l4_i2c_write_nb,
		stm32l4_i2c_read_reg,
		stm32l4_i2c_write_reg,
		stm32l4_i2c_port_get_state,
};

/**
 *
 * @param i2c_driver->i2c_vars
 * @param own_addr
 * @param default_i2c_timeout
 * @return
 */
retval_t stm32l4_i2c_init(i2c_driver_t* i2c_driver, uint8_t own_addr, uint32_t default_i2c_timeout){

	if(i2c_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}

	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);

	if(i2c_driver->i2c_vars->driver_state != DRIVER_NOT_INIT){	//Return error if the driver has been previously initiated
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
	}


	I2C_InitTypeDef* tempI2cInit = (I2C_InitTypeDef*) pvPortMalloc(sizeof(I2C_InitTypeDef));

	tempI2cInit->OwnAddress1 = own_addr;

	if(default_i2c_timeout == 0){									//Set SPI bus default timeout
		i2c_driver->i2c_vars->default_i2c_timeout = osWaitForever;
	}
	else{
		i2c_driver->i2c_vars->default_i2c_timeout = default_i2c_timeout;
	}

	switch(i2c_driver->i2c_vars->i2c_hw_number){	//The hardware i2c device used. For the yetimote STM32L4 I2C1 bus is used to control the sensors
	case 1:
		  hi2c1.Instance = I2C1;
		  hi2c1.Init.Timing = 0x2010091A;
		  hi2c1.Init.OwnAddress1 = tempI2cInit->OwnAddress1;
		  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		  hi2c1.Init.OwnAddress2 = 0;
		  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		  vPortFree(tempI2cInit);
		  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
		  {
			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_ERROR;
		  }

		  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
		  {
			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_ERROR;
		  }
		if(i2c1_semaphore_id == NULL){
			i2c1_semaphore_id = osSemaphoreCreate(osSemaphore(i2c1_semaphore), 1);
			osSemaphoreWait(i2c1_semaphore_id, osWaitForever);
		}
		break;
	default:
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		vPortFree(tempI2cInit);
		return RET_ERROR;	//Wrong hardware spi number
		break;
	}

	if(i2c_driver->i2c_vars->opened_ports == NULL){
		i2c_driver->i2c_vars->opened_ports = gen_list_init();		//Create list for the opened ports structures
	}
	i2c_driver->i2c_vars->driver_state = DRIVER_READY;

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
	return RET_OK;
}

/**
 *
 * @param i2c_driver->i2c_vars
 * @return
 */
retval_t stm32l4_i2c_deinit(i2c_driver_t* i2c_driver){

	if(i2c_driver == NULL){		//return error if the driver variables does not exist
			return RET_ERROR;
		}

	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);

	if(i2c_driver->i2c_vars->driver_state != DRIVER_READY){	//Return error if the driver has not been initiated or it is busy
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
	}

	switch(i2c_driver->i2c_vars->i2c_hw_number){
	case 1:
		if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
		{
			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_ERROR;
		}
		osSemaphoreRelease(i2c1_semaphore_id);		//Make sure we release the semaphore before destroying
		osSemaphoreRelease(i2c1_semaphore_id);
		osSemaphoreDelete(i2c1_semaphore_id);
		i2c1_semaphore_id = NULL;
		break;
	default:
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
		break;
	}

	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	i2c_port_t* temp_list_i2c_port;

	while(current->next != NULL){				//Close all opened ports for de-initializing

		temp_list_i2c_port = (i2c_port_t*) current->next->item;
		while(temp_list_i2c_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to close it(end all transmissions)

		vPortFree(temp_list_i2c_port);
		gen_list* next = current->next->next;
		vPortFree(current->next);
		current->next = next;
	}

	gen_list_remove_all(i2c_driver->i2c_vars->opened_ports);
	i2c_driver->i2c_vars->opened_ports = NULL;

	i2c_driver->i2c_vars->driver_state = DRIVER_NOT_INIT;
	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
	return RET_OK;
}

/**
 *
 * @param i2c_driver->i2c_vars
 * @param device_addr
 * @return
 */
int16_t stm32l4_i2c_open(i2c_driver_t* i2c_driver, uint8_t device_addr, i2c_port_mode_t i2c_port_mode){
	if(i2c_driver == NULL){		//return error if the driver variables does not exist
			return -1;
		}

	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);

	if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return -1;
	}

	if((i2c_port_mode != I2C_PORT_MASTER) && (i2c_port_mode != I2C_PORT_SLAVE)){
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return -1;
	}
	i2c_port_t* new_i2c_port = (i2c_port_t*) pvPortMalloc(sizeof(i2c_port_t));

	new_i2c_port->device_addr = device_addr;
	new_i2c_port->i2c_port_mode = i2c_port_mode;

	new_i2c_port->port_id = 0;								//Check the opened port list to select an ordered available port_id

	i2c_port_t* temp_list_i2c_port;
	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	uint8_t repeat_loop = 1;
	while(repeat_loop){
		repeat_loop = 0;
		current = i2c_driver->i2c_vars->opened_ports;
		while(current->next != NULL){
			temp_list_i2c_port = (i2c_port_t*) current->next->item;
			if(temp_list_i2c_port->port_id == new_i2c_port->port_id){
				repeat_loop = 1;
				new_i2c_port->port_id++;
			}
			current = current->next;
		}
	}

	new_i2c_port->port_state = DRIVER_PORT_OPENED_READY;

	gen_list_add(i2c_driver->i2c_vars->opened_ports, (void*) new_i2c_port);
	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
	return new_i2c_port->port_id;
}

/**
 *
 * @param i2c_driver->i2c_vars
 * @param port_number
 * @return
 */
retval_t stm32l4_i2c_close(i2c_driver_t* i2c_driver, int16_t port_number){
	if(i2c_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}

	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);

	if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
	}

	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	i2c_port_t* temp_list_i2c_port;

	while(current->next != NULL){

		temp_list_i2c_port = (i2c_port_t*) current->next->item;
		if(temp_list_i2c_port->port_id == port_number){

			while(temp_list_i2c_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to close it(end all transmissions)

			temp_list_i2c_port->port_id = -1;
			gen_list* next = current->next->next;
			temp_list_i2c_port->port_state = DRIVER_PORT_CLOSED;
			vPortFree(temp_list_i2c_port);
			vPortFree(current->next);
			current->next = next;

			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_OK;										//Port succesfully deleted
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
	return RET_ERROR;										//Port not found. Not deleted
}

/**
 *
 * @param i2c_driver->i2c_vars
 * @param port_number
 * @param prx
 * @param size
 * @return
 */
retval_t stm32l4_i2c_read(i2c_driver_t* i2c_driver, int16_t port_number, uint8_t *prx, uint16_t size){
	if(i2c_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);
	if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
	}

	while(i2c_driver->i2c_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	i2c_port_t* temp_list_i2c_port;
	while(current->next != NULL){

		temp_list_i2c_port = (i2c_port_t*) current->next->item;
		if(temp_list_i2c_port->port_id == port_number){

			while(temp_list_i2c_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			switch(i2c_driver->i2c_vars->i2c_hw_number){
			case 1:

				i2c_driver->i2c_vars->driver_state = DRIVER_BUSY;
				temp_list_i2c_port->port_state = DRIVER_PORT_READING;

				os_periph_dma_active = 1;

				if(temp_list_i2c_port->i2c_port_mode == I2C_PORT_MASTER){
					HAL_I2C_Master_Receive(&hi2c1, (uint16_t) temp_list_i2c_port->device_addr, prx, size, i2c_driver->i2c_vars->default_i2c_timeout);
				}
				else{
					HAL_I2C_Slave_Receive(&hi2c1, prx, size, i2c_driver->i2c_vars->default_i2c_timeout);
				}

				os_periph_dma_active = 0;
				i2c_driver->i2c_vars->driver_state = DRIVER_READY;
				temp_list_i2c_port->port_state = DRIVER_PORT_OPENED_READY;

				break;

			default:
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param i2c_driver->i2c_vars
 * @param port_number
 * @param prx
 * @param size
 * @return
 */
retval_t stm32l4_i2c_read_nb(i2c_driver_t* i2c_driver, int16_t port_number, uint8_t *prx, uint16_t size){
	if(i2c_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);
	if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
	}

	while(i2c_driver->i2c_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	i2c_port_t* temp_list_i2c_port;
	while(current->next != NULL){

		temp_list_i2c_port = (i2c_port_t*) current->next->item;
		if(temp_list_i2c_port->port_id == port_number){

			while(temp_list_i2c_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			switch(i2c_driver->i2c_vars->i2c_hw_number){
			case 1:

				i2c_driver->i2c_vars->driver_state = DRIVER_BUSY;
				temp_list_i2c_port->port_state = DRIVER_PORT_READING;

				os_periph_dma_active = 1;

				if(temp_list_i2c_port->i2c_port_mode == I2C_PORT_MASTER){
					HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t) temp_list_i2c_port->device_addr, prx, size);
				}
				else{
					HAL_I2C_Slave_Receive_DMA(&hi2c1, prx, size);
				}

				osSemaphoreWait(i2c1_semaphore_id, i2c_driver->i2c_vars->default_i2c_timeout);

				os_periph_dma_active = 0;
				i2c_driver->i2c_vars->driver_state = DRIVER_READY;
				temp_list_i2c_port->port_state = DRIVER_PORT_OPENED_READY;
				break;

			default:
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//Port id not found
	return RET_ERROR;
}


/**
 *
 * @param i2c_driver->i2c_vars
 * @param port_number
 * @param ptx
 * @param size
 * @return
 */
retval_t stm32l4_i2c_write(i2c_driver_t* i2c_driver, int16_t port_number, uint8_t *ptx, uint16_t size){
	if(i2c_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);
	if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
	}

	while(i2c_driver->i2c_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	i2c_port_t* temp_list_i2c_port;
	while(current->next != NULL){

		temp_list_i2c_port = (i2c_port_t*) current->next->item;
		if(temp_list_i2c_port->port_id == port_number){

			while(temp_list_i2c_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			switch(i2c_driver->i2c_vars->i2c_hw_number){
			case 1:

				i2c_driver->i2c_vars->driver_state = DRIVER_BUSY;
				temp_list_i2c_port->port_state = DRIVER_PORT_READING;

				os_periph_dma_active = 1;

				if(temp_list_i2c_port->i2c_port_mode == I2C_PORT_MASTER){
					HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) temp_list_i2c_port->device_addr, ptx, size, i2c_driver->i2c_vars->default_i2c_timeout);
				}
				else{
					HAL_I2C_Slave_Transmit(&hi2c1, ptx, size, i2c_driver->i2c_vars->default_i2c_timeout);
				}

				os_periph_dma_active = 0;
				i2c_driver->i2c_vars->driver_state = DRIVER_READY;
				temp_list_i2c_port->port_state = DRIVER_PORT_OPENED_READY;

				break;

			default:
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param spi_variables
 * @param port_number
 * @param ptx
 * @param size
 * @return
 */
retval_t stm32l4_i2c_write_nb(i2c_driver_t* i2c_driver, int16_t port_number, uint8_t *ptx, uint16_t size){
	if(i2c_driver == NULL){		//return error if the driver variables does not exist
			return RET_ERROR;
		}
		osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);
		if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_ERROR;
		}

		while(i2c_driver->i2c_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

		gen_list* current = i2c_driver->i2c_vars->opened_ports;
		i2c_port_t* temp_list_i2c_port;
		while(current->next != NULL){

			temp_list_i2c_port = (i2c_port_t*) current->next->item;
			if(temp_list_i2c_port->port_id == port_number){

				while(temp_list_i2c_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

				switch(i2c_driver->i2c_vars->i2c_hw_number){
				case 1:

					i2c_driver->i2c_vars->driver_state = DRIVER_BUSY;
					temp_list_i2c_port->port_state = DRIVER_PORT_READING;

					os_periph_dma_active = 1;

					if(temp_list_i2c_port->i2c_port_mode == I2C_PORT_MASTER){
						HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t) temp_list_i2c_port->device_addr, ptx, size);
					}
					else{
						HAL_I2C_Slave_Transmit_DMA(&hi2c1, ptx, size);
					}

					osSemaphoreWait(i2c1_semaphore_id, i2c_driver->i2c_vars->default_i2c_timeout);

					os_periph_dma_active = 0;
					i2c_driver->i2c_vars->driver_state = DRIVER_READY;
					temp_list_i2c_port->port_state = DRIVER_PORT_OPENED_READY;
					break;

				default:
					osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//INVALID HW DEVICE
					return RET_ERROR;
					break;
				}

				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
				return RET_OK;										//Data successfully sent
			}

			else{
				current = current->next;
			}

		}

		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//Port id not found
		return RET_ERROR;
}

/**
 *
 * @param i2c_driver->i2c_vars
 * @param port_number
 * @param reg_addr
 * @param reg_addr_size
 * @param prx
 * @param size
 * @return
 */
retval_t stm32l4_i2c_read_reg(i2c_driver_t* i2c_driver, int16_t port_number, uint16_t reg_addr, uint16_t reg_addr_size, uint8_t *prx, uint16_t size){
	if(i2c_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);
	if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
	}

	while(i2c_driver->i2c_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	i2c_port_t* temp_list_i2c_port;
	while(current->next != NULL){

		temp_list_i2c_port = (i2c_port_t*) current->next->item;
		if(temp_list_i2c_port->port_id == port_number){

			while(temp_list_i2c_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			if(temp_list_i2c_port->i2c_port_mode != I2C_PORT_MASTER){
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);							//MEMORY READ WRITE ONLY ALLOWED IN MASTER MODE
				return RET_ERROR;
			}
			if((reg_addr_size != 1) && (reg_addr_size != 2)){
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);							//1 or 2 bytes for memory allowed
				return RET_ERROR;
			}

			switch(i2c_driver->i2c_vars->i2c_hw_number){
			case 1:

				i2c_driver->i2c_vars->driver_state = DRIVER_BUSY;
				temp_list_i2c_port->port_state = DRIVER_PORT_READING;

				os_periph_dma_active = 1;


				HAL_I2C_Mem_Read(&hi2c1, (uint16_t) temp_list_i2c_port->device_addr, reg_addr, reg_addr_size, prx, size, i2c_driver->i2c_vars->default_i2c_timeout);

//				osSemaphoreWait(i2c1_semaphore_id, i2c_driver->i2c_vars->default_i2c_timeout);

				os_periph_dma_active = 0;
				i2c_driver->i2c_vars->driver_state = DRIVER_READY;
				temp_list_i2c_port->port_state = DRIVER_PORT_OPENED_READY;
				break;

			default:
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//Port id not found
	return RET_ERROR;
}

/**
 *
 * @param i2c_driver->i2c_vars
 * @param port_number
 * @param reg_addr
 * @param reg_addr_size
 * @param ptx
 * @param size
 * @return
 */
retval_t stm32l4_i2c_write_reg(i2c_driver_t* i2c_driver, int16_t port_number, uint16_t reg_addr, uint16_t reg_addr_size, uint8_t *ptx, uint16_t size){
	if(i2c_driver == NULL){		//return error if the driver variables does not exist
		return RET_ERROR;
	}
	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);
	if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return RET_ERROR;
	}

	while(i2c_driver->i2c_vars->driver_state == DRIVER_BUSY);			//Wait till the spi driver is ready

	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	i2c_port_t* temp_list_i2c_port;
	while(current->next != NULL){

		temp_list_i2c_port = (i2c_port_t*) current->next->item;
		if(temp_list_i2c_port->port_id == port_number){

			while(temp_list_i2c_port->port_state !=  DRIVER_PORT_OPENED_READY);			//Wait till the port is ready to read(end all transmissions)

			if(temp_list_i2c_port->i2c_port_mode != I2C_PORT_MASTER){
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);							//MEMORY READ WRITE ONLY ALLOWED IN MASTER MODE
				return RET_ERROR;
			}
			if((reg_addr_size != 1) && (reg_addr_size != 2)){
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);							//1 or 2 bytes for memory allowed
				return RET_ERROR;
			}

			switch(i2c_driver->i2c_vars->i2c_hw_number){
			case 1:

				i2c_driver->i2c_vars->driver_state = DRIVER_BUSY;
				temp_list_i2c_port->port_state = DRIVER_PORT_READING;

				os_periph_dma_active = 1;

				HAL_I2C_Mem_Write(&hi2c1, (uint16_t) temp_list_i2c_port->device_addr, reg_addr, reg_addr_size, ptx, size, i2c_driver->i2c_vars->default_i2c_timeout);

//				osSemaphoreWait(i2c1_semaphore_id, i2c_driver->i2c_vars->default_i2c_timeout);

				os_periph_dma_active = 0;
				i2c_driver->i2c_vars->driver_state = DRIVER_READY;
				temp_list_i2c_port->port_state = DRIVER_PORT_OPENED_READY;
				break;

			default:
				osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//INVALID HW DEVICE
				return RET_ERROR;
				break;
			}

			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
			return RET_OK;										//Data successfully sent
		}

		else{
			current = current->next;
		}

	}

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//Port id not found
	return RET_ERROR;
}


/**
 *
 * @param i2c_driver->i2c_vars
 * @param port_number
 * @return
 */
port_state_t stm32l4_i2c_port_get_state(i2c_driver_t* i2c_driver, int16_t port_number){

	if(i2c_driver == NULL){		//return error if the driver variables does not exist
		return DRIVER_PORT_CLOSED;
	}
	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);
	if(i2c_driver->i2c_vars->driver_state == DRIVER_NOT_INIT){		//Return error if the driver is not initialized
		osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);
		return DRIVER_PORT_CLOSED;
	}

	gen_list* current = i2c_driver->i2c_vars->opened_ports;
	i2c_port_t* temp_list_i2c_port;
	while(current->next != NULL){

		temp_list_i2c_port = (i2c_port_t*) current->next->item;
		if(temp_list_i2c_port->port_id == port_number){

			port_state_t port_state = temp_list_i2c_port->port_state;
			osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);

			return port_state;										//return port state
		}

		else{
			current = current->next;
		}
	}

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);						//Port id not found
	return DRIVER_PORT_CLOSED;
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
		osSemaphoreRelease(i2c1_semaphore_id);
	}
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
		osSemaphoreRelease(i2c1_semaphore_id);
	}
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
		osSemaphoreRelease(i2c1_semaphore_id);
	}
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
		osSemaphoreRelease(i2c1_semaphore_id);
	}
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
		osSemaphoreRelease(i2c1_semaphore_id);
	}
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
		osSemaphoreRelease(i2c1_semaphore_id);
	}
}
