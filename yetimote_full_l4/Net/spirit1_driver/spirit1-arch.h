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
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
#ifndef __SPIRIT1_ARCH_H__
#define __SPIRIT1_ARCH_H__
/*---------------------------------------------------------------------------*/
#include "string.h"
#include "SPIRIT_Management.h"
#include "spirit1-config.h"
/*---------------------------------------------------------------------------*/
#define IRQ_ENABLE_433()             EnableIRQ(SPIRIT_433_GPIO_IRQ,SPIRIT_433_EXT_INTERRUPT, 9, 0)
#define IRQ_DISABLE_433()            DisableIRQ(SPIRIT_433_GPIO_IRQ,SPIRIT_433_EXT_INTERRUPT, 9, 0)

#define IRQ_ENABLE_868()             EnableIRQ(SPIRIT_868_GPIO_IRQ,SPIRIT_868_EXT_INTERRUPT, 9, 0)
#define IRQ_DISABLE_868()            DisableIRQ(SPIRIT_868_GPIO_IRQ,SPIRIT_868_EXT_INTERRUPT, 9, 0)

#define SPIRIT1_STATUS()       	(spirit1_arch_refresh_status() & SPIRIT1_STATE_STATEBITS)
#define OS_GET_TIME()			osKernelSysTick()

/*---------------------------------------------------------------------------*/

/* Input GPIO for Spirit1 interrupt */
#define SPIRIT_GPIO_IRQ         	SPIRIT_GPIO_3

/*---------------------------------------------------------------------------*/

/* SPIRIT1_Spi_config_Headers */
#define HEADER_WRITE_MASK     0x00                                /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01                                /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00                                /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80                                /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS   0xFF                                  /*!< Linear FIFO address*/

#define SPIRIT1_IRQ_ENABLE()   IRQ_ENABLE_433();IRQ_ENABLE_868();
#define SPIRIT1_IRQ_DISABLE()  IRQ_DISABLE_433();IRQ_DISABLE_868();

/* SPIRIT1_Spi_config_Private_FunctionPrototypes */
#define SPI_ENTER_CRITICAL()  SPIRIT1_IRQ_DISABLE()
#define SPI_EXIT_CRITICAL()   SPIRIT1_IRQ_ENABLE()

/* SPIRIT1_Spi_config_Private_Functions */
#define RadioSpiCSLow_433()        HAL_GPIO_WritePin(SPIRIT_433_SPI_CS_PORT, SPIRIT_433_SPI_CS_PIN, GPIO_PIN_RESET)
#define RadioSpiCSHigh_433()       HAL_GPIO_WritePin(SPIRIT_433_SPI_CS_PORT, SPIRIT_433_SPI_CS_PIN, GPIO_PIN_SET)
#define spirit_spi_busy_433()       (!(SPIRIT_433_SPI_CS_PORT->IDR & SPIRIT_433_SPI_CS_PIN))

#define RadioSpiCSLow_868()        HAL_GPIO_WritePin(SPIRIT_868_SPI_CS_PORT, SPIRIT_868_SPI_CS_PIN, GPIO_PIN_RESET)
#define RadioSpiCSHigh_868()       HAL_GPIO_WritePin(SPIRIT_868_SPI_CS_PORT, SPIRIT_868_SPI_CS_PIN, GPIO_PIN_SET)
#define spirit_spi_busy_868()       (!(SPIRIT_868_SPI_CS_PORT->IDR & SPIRIT_868_SPI_CS_PIN))


/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...) */
#define RADIO_SPI_TIMEOUT_MAX                   ((uint32_t)1000)

/* SPIRIT1_Spi_config_Private_Macros */
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)                             /*!< macro to build the header byte*/
#define WRITE_HEADER        BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write
                                                                                                         header byte*/
#define READ_HEADER         BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read
                                                                                                         header byte*/
#define COMMAND_HEADER      BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command
                                                                                                         header byte*/


SpiritStatus RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
SpiritStatus RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer);
SpiritStatus RadioSpiCommandStrobes(uint8_t cCommandCode);

/*---------------------------------------------------------------------------*/
uint16_t spirit1_arch_refresh_status(void);
void EnableIRQ(uint16_t gpio, IRQn_Type line, uint8_t nPreemption, uint8_t nSubpriority);
void DisableIRQ(uint16_t gpio, IRQn_Type line, uint8_t nPreemption, uint8_t nSubpriority);
/*---------------------------------------------------------------------------*/
#endif /* __SPIRIT1_ARCH_H__ */
