/*
 * Copyright (c) 2012, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */
/*---------------------------------------------------------------------------*/
#include "stm32l4xx.h"
#include "spirit1-arch.h"
#include "spirit1.h"
#include "MCU_Interface.h"
#include "netstack.h"
#include "yetimote-conf.h"
#include "os_spi.h"
/*---------------------------------------------------------------------------*/
//extern void spirit1_interrupt_callback(void);
SpiritBool spiritdk_timer_expired = S_FALSE;
/*---------------------------------------------------------------------------*/
/* use the SPI-port to acquire the status bytes from the radio. */
#define CS_TO_SCLK_DELAY  0x0100

/*---------------------------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
uint32_t SpiTimeout = RADIO_SPI_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */
/*---------------------------------------------------------------------------*/

extern spi_driver_t* spi2_driver;
int16_t port_id_868;
int16_t port_id_433;

uint8_t spi_done = 0;

extern radio_driver_state_t current_radio_driver;

/**
 * @brief Updates the spirit1 status
 * @return the spirit1 state
 */
uint16_t spirit1_arch_refresh_status(void) {
	volatile uint16_t mcstate = 0x0000;
	volatile uint8_t tmpStatus[2];
	uint8_t header[2];
	header[0] = 0x01;
	header[1] = MC_STATE1_BASE;

	/* Sets the length of the packet to send */
	  if(current_radio_driver == RADIO_SPIRIT_433){
		IRQ_DISABLE_433();
	  }
	  else if(current_radio_driver == RADIO_SPIRIT_868){
		IRQ_DISABLE_868();
	  }

	if(current_radio_driver == RADIO_SPIRIT_433){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_433);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_433,(uint8_t *) &header[0], (uint8_t *) &tmpStatus, 2);
		}
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_868);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_868,(uint8_t *) &header[0], (uint8_t *) &tmpStatus, 2);
		}
	}

	mcstate = (tmpStatus[0]<<8) + tmpStatus[1];
	  /* Sets the length of the packet to send */
	  if(current_radio_driver == RADIO_SPIRIT_433){
		IRQ_ENABLE_433();
	  }
	  else if(current_radio_driver == RADIO_SPIRIT_868){
		IRQ_ENABLE_868();
	  }

	return mcstate;
}


/**
 * @brief  Enable an interrupt
 * @param  gpio GPIO pin
 * @param  line: External interrupt number
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32l1xx.h))
 * @param  nPreemption: The pre-emption priority for the IRQn channel.
 *         This parameter can be a value between 0 and 15
 *         A lower priority value indicates a higher priority
 * @param  nSubpriority: the subpriority level for the IRQ channel.
 *         This parameter can be a value between 0 and 15
 *         A lower priority value indicates a higher priority.
 */
void EnableIRQ(uint16_t gpio, IRQn_Type line, uint8_t nPreemption, uint8_t nSubpriority) {
	__HAL_GPIO_EXTI_CLEAR_IT(gpio);
	HAL_NVIC_SetPriority(line, nPreemption, nSubpriority);	//ENABLE INTERRUPT
	HAL_NVIC_EnableIRQ(line);
}

/**
 * @brief  Disable an interrupt
 * @param  gpio GPIO pin
 * @param  line: External interrupt number
 *         This parameter can be an enumerator of IRQn_Type enumeration
 *         (For the complete STM32 Devices IRQ Channels list, please refer to the appropriate CMSIS device file (stm32l1xx.h))
 * @param  nPreemption: The pre-emption priority for the IRQn channel.
 *         This parameter can be a value between 0 and 15
 *         A lower priority value indicates a higher priority
 * @param  nSubpriority: the subpriority level for the IRQ channel.
 *         This parameter can be a value between 0 and 15
 *         A lower priority value indicates a higher priority.
 */
void DisableIRQ(uint16_t gpio, IRQn_Type line, uint8_t nPreemption,
		uint8_t nSubpriority) {
	__HAL_GPIO_EXTI_CLEAR_IT(gpio);
	HAL_NVIC_SetPriority(line, nPreemption, nSubpriority);	//ENABLE INTERRUPT
	HAL_NVIC_DisableIRQ(line);

}

/**
 * @brief  Initializes SPI HAL.
 * @param  None
 * @retval None
 */
void RadioSpiInit(void) {

	if(current_radio_driver == RADIO_SPIRIT_433){
		port_id_433 = spi2_driver->spi_funcs.spi_open(spi2_driver,ENABLE_SW_CS, SPIRIT_433_SPI_CS_PIN_OS_DRIVER, SPIRIT_433_SPI_CS_PORT_OS_DRIVER);
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		port_id_868 = spi2_driver->spi_funcs.spi_open(spi2_driver,ENABLE_SW_CS, SPIRIT_868_SPI_CS_PIN_OS_DRIVER, SPIRIT_868_SPI_CS_PORT_OS_DRIVER);
	}
}

/**
 *
 */
void RadioSpiClose(void) {

	if(current_radio_driver == RADIO_SPIRIT_433){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_433);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_close(spi2_driver,port_id_433);
		}
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_868);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_close(spi2_driver,port_id_868);
		}
	}
}

/**
 * @brief  Write single or multiple RF Transceivers register
 * @param  cRegAddress: base register's address to be write
 * @param  cNbBytes: number of registers and bytes to be write
 * @param  pcBuffer: pointer to the buffer of values have to be written into registers
 * @retval SpiritStatus
 */
SpiritStatus RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes,
		uint8_t* pcBuffer) {
//	uint8_t aHeader[2] = { 0 };

	uint8_t* tmpRx = (uint8_t*) pvPortMalloc(cNbBytes+2);
	uint8_t* tmpTx = (uint8_t*) pvPortMalloc(cNbBytes+2);

	memcpy(tmpTx+2, pcBuffer, cNbBytes);
	tmpTx[0] = WRITE_HEADER;
	tmpTx[1] = cRegAddress;
	uint8_t tmpRetStatus[2];

	SpiritStatus* pStatus = &tmpRetStatus[0];

	SPI_ENTER_CRITICAL();

	if(current_radio_driver == RADIO_SPIRIT_433){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_433);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_433,(uint8_t *) tmpTx, (uint8_t *) tmpRx, cNbBytes + 2);
		}
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_868);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_868,(uint8_t *) tmpTx, (uint8_t *) tmpRx, cNbBytes + 2);
		}
	}

	tmpRetStatus[0] = tmpRx[1];
	tmpRetStatus[1] = tmpRx[0];

	SPI_EXIT_CRITICAL();

	vPortFree(tmpRx);
	vPortFree(tmpTx);

	return *pStatus;
}

/**
 * @brief  Read single or multiple SPIRIT1 register
 * @param  cRegAddress: base register's address to be read
 * @param  cNbBytes: number of registers and bytes to be read
 * @param  pcBuffer: pointer to the buffer of registers' values read
 * @retval SpiritStatus
 */
SpiritStatus RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes,
		uint8_t* pcBuffer) {

	uint8_t* tmpRx = (uint8_t*) pvPortMalloc(cNbBytes+2);
	uint8_t* tmpTx = (uint8_t*) pvPortMalloc(cNbBytes+2);

	tmpTx[0] = READ_HEADER;
	tmpTx[1] = cRegAddress;
	uint8_t tmpRetStatus[2];

	SpiritStatus* pStatus = &tmpRetStatus[0];

	SPI_ENTER_CRITICAL();

	if(current_radio_driver == RADIO_SPIRIT_433){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_433);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_433,(uint8_t *) tmpTx, (uint8_t *) tmpRx, cNbBytes + 2);
		}
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_868);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_868,(uint8_t *) tmpTx, (uint8_t *) tmpRx, cNbBytes + 2);
		}
	}

	tmpRetStatus[0] = tmpRx[1];
	tmpRetStatus[1] = tmpRx[0];

	SPI_EXIT_CRITICAL();

	memcpy(pcBuffer, tmpRx+2, cNbBytes);

	vPortFree(tmpRx);
	vPortFree(tmpTx);

	return *pStatus;

}

/**
 * @brief  Send a command
 * @param  cCommandCode: command code to be sent
 * @retval SpiritStatus
 */
SpiritStatus RadioSpiCommandStrobes(uint8_t cCommandCode) {
	uint8_t aHeader[2] = { 0 };
	uint8_t tmpStatus[2];
	uint8_t tmpRetStatus[2];
	SpiritStatus* pStatus = &tmpRetStatus[0];

	/* Built the aHeader bytes */
	aHeader[0] = COMMAND_HEADER;
	aHeader[1] = cCommandCode;

	SPI_ENTER_CRITICAL();

	if(current_radio_driver == RADIO_SPIRIT_433){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_433);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_433,(uint8_t *) &aHeader[0], (uint8_t *) tmpStatus, 2);
		}
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_868);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_868,(uint8_t *) &aHeader[0], (uint8_t *) tmpStatus, 2);
		}
	}

	tmpRetStatus[0] = tmpStatus[1];
	tmpRetStatus[1] = tmpStatus[0];

	SPI_EXIT_CRITICAL();

	return *pStatus;
}

/**
 * @brief  Write data into TX FIFO
 * @param  cNbBytes: number of bytes to be written into TX FIFO
 * @param  pcBuffer: pointer to data to write
 * @retval SpiritStatus
 */
SpiritStatus RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
	uint8_t* tmpRx = (uint8_t*) pvPortMalloc(cNbBytes+2);
	uint8_t* tmpTx = (uint8_t*) pvPortMalloc(cNbBytes+2);

	memcpy(tmpTx+2, pcBuffer, cNbBytes);

	tmpTx[0] = WRITE_HEADER;
	tmpTx[1] = LINEAR_FIFO_ADDRESS;
	uint8_t tmpRetStatus[2];

	SpiritStatus* pStatus = &tmpRetStatus[0];

	SPI_ENTER_CRITICAL();

//	/* Put the SPI chip select high to end the transaction */
	if(current_radio_driver == RADIO_SPIRIT_433){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_433);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_433,(uint8_t *) tmpTx, (uint8_t *) tmpRx, cNbBytes + 2);
		}
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_868);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_868,(uint8_t *) tmpTx, (uint8_t *) tmpRx, cNbBytes + 2);
		}
	}



	tmpRetStatus[0] = tmpRx[1];
	tmpRetStatus[1] = tmpRx[0];

	SPI_EXIT_CRITICAL();

	vPortFree(tmpRx);
	vPortFree(tmpTx);

	return *pStatus;
}

/**
 * @brief  Read data from RX FIFO
 * @param  cNbBytes: number of bytes to read from RX FIFO
 * @param  pcBuffer: pointer to data read from RX FIFO
 * @retval SpiritStatus
 */
SpiritStatus RadioSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
	uint8_t* tmpRx = (uint8_t*) pvPortMalloc(cNbBytes+2);
	uint8_t* tmpTx = (uint8_t*) pvPortMalloc(cNbBytes+2);

	memcpy(tmpTx+2, pcBuffer, cNbBytes);

	tmpTx[0] = READ_HEADER;
	tmpTx[1] = LINEAR_FIFO_ADDRESS;
	uint8_t tmpRetStatus[2];

	SpiritStatus* pStatus = &tmpRetStatus[0];

	SPI_ENTER_CRITICAL();

	if(current_radio_driver == RADIO_SPIRIT_433){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_433);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_433,(uint8_t *) tmpTx, (uint8_t *) tmpRx, cNbBytes + 2);
		}
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		port_state_t spi_state = spi2_driver->spi_funcs.spi_port_get_state(spi2_driver, port_id_868);
		if(spi_state != DRIVER_PORT_CLOSED){
			spi2_driver->spi_funcs.spi_read_write_nb(spi2_driver, port_id_868,(uint8_t *) tmpTx, (uint8_t *) tmpRx, cNbBytes + 2);
		}
	}

	tmpRetStatus[0] = tmpRx[1];
	tmpRetStatus[1] = tmpRx[0];

	SPI_EXIT_CRITICAL();

	memcpy(pcBuffer, tmpRx+2, cNbBytes);

	vPortFree(tmpRx);
	vPortFree(tmpTx);

	return *pStatus;
}

void RadioEnterShutdown(void){}

void RadioExitShutdown(void){}
