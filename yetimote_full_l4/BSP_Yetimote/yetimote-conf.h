/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         A brief description of what this file is
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

#ifndef PLATFORM_CONF_H_
#define PLATFORM_CONF_H_


#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "types.h"
#include "FreeRTOSConfig.h"
//#include "core_cmSimd.h"

/*
 * Definitions below are dictated by the hardware and not really
 * changeable!
 */

#if DEBUG_USB
#define usb_printf(...) printf(__VA_ARGS__)	//Usado para sacar trazas por USB. Cuando esta activo los modos bajo consumo no funcionan.
#else
#define usb_printf(...)
#endif
/* LED ports */
/* LED ports */
#define LEDS_PORT_B	GPIOB
#define LEDS_BLUE_PIN	GPIO_PIN_5
#define LEDS_GREEN_PIN	GPIO_PIN_4
#define LEDS_PORT_A	GPIOA
#define LEDS_RED1_PIN	GPIO_PIN_8
#define LEDS_RED2_PIN	GPIO_PIN_15

/* Button ports */
#define BUTTON_PORT	GPIOB
#define BUTTON_PIN	GPIO_PIN_1

/* Acelerometer */
#define ACEL_USE_INT_PIN	0
#define ACELEROMETER_PORT	GPIOB
#define ACELEROMETER_PIN	GPIO_PIN_9

/* USE THE HW RTC FOR TIMING */
#define USE_RTC 1

/* PERIPHERAL DRIVERS */
#define FULL_DEVICE_DRIVER	0
#define OS_SPI_DRIVER_FUNCS		stm32l4_spi_funcs
#define OS_I2C_DRIVER_FUNCS		stm32l4_i2c_funcs

/* GPIO control pins for Spirit1-433 */
#define SPIRIT_433_GPIO_IRQ					GPIO_PIN_4
#define SPIRIT_433_GPIO_IRQ_PORT			GPIOC
#define SPIRIT_433_EXT_INTERRUPT			EXTI4_IRQn
#define SPIRIT_433_GPIO_SDN_PIN         	GPIO_PIN_7
#define SPIRIT_433_GPIO_SDN_PORT        	GPIOC
#define SPIRIT_433_SPI_CS_PIN           	GPIO_PIN_12
#define SPIRIT_433_SPI_CS_PORT          	GPIOB
#define SPIRIT_433_SPI_CS_PIN_OS_DRIVER    	12		//GPIO_PIN_12
#define SPIRIT_433_SPI_CS_PORT_OS_DRIVER   	1		//GPIOB

/* GPIO control pins for Spirit1-868 */
#define SPIRIT_868_GPIO_IRQ					GPIO_PIN_2
#define SPIRIT_868_GPIO_IRQ_PORT			GPIOC
#define SPIRIT_868_EXT_INTERRUPT			EXTI2_IRQn
#define SPIRIT_868_GPIO_SDN_PIN         	GPIO_PIN_6
#define SPIRIT_868_GPIO_SDN_PORT        	GPIOC
#define SPIRIT_868_SPI_CS_PIN           	GPIO_PIN_0
#define SPIRIT_868_SPI_CS_PORT          	GPIOB
#define SPIRIT_868_SPI_CS_PIN_OS_DRIVER     0		//GPIO_PIN_0
#define SPIRIT_868_SPI_CS_PORT_OS_DRIVER    1		//GPIOB


/* GPIO control pins for external CC2500 */
#define CC2500_GDO2_IRQ_PIN			GPIO_PIN_0
#define CC2500_GDO2_IRQ_PORT		GPIOC
#define CC2500_EXT_INTERRUPT		EXTI0_IRQn
#define CC2500_SPI_CS_PIN           GPIO_PIN_2
#define CC2500_SPI_CS_PORT          GPIOB
#define CC2500_SPI					SPI2

/* SPI ports */
#define SPIRIT_SPI                   SPI2



/*NODE ID LOCATION STM32L1*/
#define NODEID_LOCATION_BASE ((uint32_t*)0x1FFF7590)
#define NODEID_OFFSET0	0x00
#define NODEID_OFFSET1	0x01
#define NODEID_OFFSET2	0x02

#define USE_RADIO	1

#define SYSTEM_CONF_SECOND	1000		//Number of systicks in a second


//Size of the network addresses


#define LINKADDR_CONF_SIZE 2

#define PACKETBUF_CONF_SIZE 	256
#define PACKETBUF_CONF_HDR_SIZE 48

#define SPIRIT_DRIVER_802154_AUTOACK	1

#define USE_RADIO	1
#define NETSTACK_CC2500_RADIO	cc2500_driver
#define NETSTACK_SPIRIT_RADIO	spirit_radio_driver
#define NETSTACK_CONF_RADIO 	spirit_radio_driver
#define NETSTACK_CONF_RDC     	nullrdc_driver
#define NETSTACK_CONF_MAC     	nullmac_driver
#define NETSTACK_CONF_LLSEC 	nullsec_driver

#define NETSTACK_CONF_FRAMER 	framer_802154
//#define LLSEC802154_CONF_SECURITY_LEVEL	FRAME802154_SECURITY_LEVEL_ENC_MIC_128


#define NETSTACK_CONF_NETWORK 	rime_driver
#define NETSTACK_CONF_WITH_RIME	1

#define ROUTE_CONF_ENTRIES			10				//Numero de entradas máximas de la tabla de rutas

#define QUEUEBUF_CONF_NUM			4

/*Interfaz radio usada inicialmente*/
#define INIT_HW_RADIO_DRIVER	RADIO_CC2500	//RADIO_SPIRIT_433, RADIO_SPIRIT_868, RADIO_CC2500
#define	INIT_SW_RADIO_DRIVER	NETSTACK_CC2500_RADIO	//NETSTACK_SPIRIT_RADIO, NETSTACK_CC2500_RADIO

/* Functions declarations to avoid math DSP warnings */
//void __SMUAD(void);
//void __SMLALD(void);
//void __SSAT(void);
//void __QADD(void);
//void __QSUB(void);

#endif /* PLATFORM_CONF_H_ */
