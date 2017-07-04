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
 * os_spi.h
 *
 *  Created on: 16 de ene. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file os_spi.h
 */

#ifndef APPLICATION_USER_BSP_YETIMOTE_PERIPHERAL_OS_SPI_H_
#define APPLICATION_USER_BSP_YETIMOTE_PERIPHERAL_OS_SPI_H_

#include "yetimote-conf.h"
#include "periph_drivers.h"
#include "generic_list.h"

#define ENABLE_SW_CS	1
#define DISABLE_SW_CS	0

typedef enum{
	SPI_SLAVE,				//!> SPI slave mode
	SPI_MASTER,				//!> SPI master mode
}spi_mode_t;

typedef enum{
	LINES_RX,				//!> SPI only RX mode
	LINES_DUPLEX,			//!> SPI full duplex Rx/Tx mode
}spi_lines_t;

typedef enum{
	SPI_POL_HIGH,			//!> SPI Polarity high
	SPI_POL_LOW,			//!> SPI Polarity low
}spi_polarity_t;

typedef enum{
	SPI_1EDGE,				//!> SPI Phase 1 edge
	SPI_2EDGE,				//!> SPI Phase 2 edge
}spi_phase_t;

typedef enum{
	SPI_SOFT_NSS,			//!> SPI Software NSS signal
	SPI_HW_NSS_INPUT,		//!> SPI Hardware NSS input signal
	SPI_HW_NSS_OUTPUT,		//!> SPI Hardware NSS output signal
}spi_nss_mode_t;

/**
 * @brief	Struct containing a specific virtual port configuration variables for SPI
 */
typedef struct spi_port_{
	int16_t port_id;			//!> Virtual port id opened
	uint16_t cs_hw_pin;			//!> Virtual cs_hw_pin. It should correspond to the real HW PIN instance
	uint16_t cs_hw_port;		//!> Virtual cs_hw_port. It should correspond to the real HW PORT instance
	port_state_t port_state;	//!> Current state of this spi port
	uint8_t sw_cs;				//!> Enables or disables the software chip select of this virtual port
}spi_port_t;

/**
 * @brief Struct containing the variables used by an SPI driver
 */
typedef struct spi_vars_{
	uint8_t spi_hw_number;			//!> Number of HW SPI instance

	spi_mode_t	spi_mode;			//!> SPi mode: slave or master
	spi_lines_t spi_lines;			//!> SPI line number: only rx mode or duplex rx/tx mode
	uint8_t	data_size;				//!> Size of each data, from 4 to 16 bits
	spi_polarity_t spi_polarity;	//!> Spi polarity. High or Low
	spi_phase_t spi_phase;			//!> Spi phase.1 Edge or 2 Edge
	spi_nss_mode_t spi_nss;			//!> Spi nss signal. SW or HW
	uint32_t spi_speed;				//!> Spi bus speed
	uint32_t default_spi_timeout;	//!> SPI read write timeout. If 0 it means an infinite timeout

	gen_list* opened_ports;			//!> List with the ports opened. Each element in the list is a spi_port_t type element.

	driver_state_t driver_state;	//!> Current state of the spi driver
	osMutexId spi_mutex_id;			//!> Spi driver mutex. Only one thread can use driver functions at a time
}spi_vars_t;

typedef struct spi_funcs_ spi_funcs_t;
#if SPI_DRIVER_CONFIG_FUNCS
typedef	struct spi_config_funcs_ spi_config_funcs_t;
#endif


/**
 * @brief	SPI Driver object. It must be an object for each physical SPI bus.
 */
typedef struct spi_driver_{
	spi_vars_t* spi_vars;						//!> SPI driver variables

	struct spi_funcs_{
		retval_t (*spi_init)(struct spi_driver_* spi_driver, spi_mode_t	spi_mode, spi_lines_t spi_lines, uint8_t data_size, spi_polarity_t spi_polarity,
				spi_phase_t spi_phase, spi_nss_mode_t spi_nss, uint32_t spi_speed, uint32_t default_spi_timeout);				//!> Initialize and configures a hardware SPI device. It must configure all HW parameters of the SPI
		retval_t (*spi_deinit)(struct spi_driver_* spi_driver);																		//!> Disable an initialized hardware SPI device
		int16_t (*spi_open)(struct spi_driver_* spi_driver, uint8_t enable_sw_cs, uint16_t cs_hw_pin, uint16_t cs_hw_port);			//!> Open a virtual port associated with a sw chip select. It returns the virtual port id opened (-1 if error)
		retval_t (*spi_close)(struct spi_driver_* spi_driver, int16_t port_number);													//!> Close a previously opened virtual port
		retval_t (*spi_read)(struct spi_driver_* spi_driver, int16_t port_number, uint8_t *prx, uint16_t size);						//!> Read from spi the specified bytes number. It is a blocking function. It blocks till completed or spi timeout reached
		retval_t (*spi_read_nb)(struct spi_driver_* spi_driver, int16_t port_number, uint8_t *prx, uint16_t size);					//!> Read from spi the specified bytes number in non blocking mode. The current thread sleeps till completed or spi timeout reached
		retval_t (*spi_write)(struct spi_driver_* spi_driver, int16_t port_number, uint8_t *ptx, uint16_t size);						//!> Write to spi the specified bytes number. It is a blocking function. It blocks till completed or spi timeout reached
		retval_t (*spi_write_nb)(struct spi_driver_* spi_driver, int16_t port_number, uint8_t *ptx, uint16_t size);					//!> Write from spi the specified bytes number in non blocking mode. The current thread sleeps till completed or spi timeout reached
		retval_t (*spi_read_write)(struct spi_driver_* spi_driver, int16_t port_number, uint8_t *ptx, uint8_t *prx, uint16_t size);	//!> Duplex Read/Write function the specified bytes number. It blocks till completed or spi timeout reached. Duplex mode must be initiated
		retval_t (*spi_read_write_nb)(struct spi_driver_* spi_driver, int16_t port_number, uint8_t *ptx, uint8_t *prx, uint16_t size); //!> Duplex Read/Write function the specified bytes number in non blocking mode. The current thread sleeps till completed or spi timeout reached. Duplex mode must be initiated
		port_state_t (*spi_port_get_state)(struct spi_driver_* spi_driver, int16_t port_number); 										//!> Returns the current state of the specified port
	}spi_funcs;

#if SPI_DRIVER_CONFIG_FUNCS
	struct spi_config_funcs_{
		retval_t (*spi_set_mode)(struct spi_driver_* spi_driver, spi_mode_t	spi_mode);				//!> Set SPI mode master or slave
		retval_t (*spi_set_lines)(struct spi_driver_* spi_driver, spi_lines_t spi_lines);			//!> Set SPI lines mode. Only RX or TX/RX Duplex
		retval_t (*spi_set_data_size)(struct spi_driver_* spi_driver, uint8_t data_size);			//!> Set SPI data size, from 4 to 16 bits
		retval_t (*spi_set_polarity)(struct spi_driver_* spi_driver, spi_polarity_t spi_polarity);	//!> Set SPI polarity, high or low
		retval_t (*spi_set_phase)(struct spi_driver_* spi_driver, spi_phase_t spi_phase);			//!> Set SPI phase, 1 edge or 2 edge
		retval_t (*spi_set_nss_mode)(struct spi_driver_* spi_driver, spi_nss_mode_t spi_nss);		//!> Set SPI NSS signal mode, HW or SW
		retval_t (*spi_set_speed)(struct spi_driver_* spi_driver, uint32_t spi_speed);				//!> Set SPI bus speed
		retval_t (*spi_set_timeout)(struct spi_driver_* spi_driver, uint32_t default_spi_timeout);	//!> Set SPI operations timeout
		spi_mode_t (*spi_get_mode)(struct spi_driver_* spi_driver);									//!> Get SPI mode master or slave
		spi_lines_t (*spi_get_lines)(struct spi_driver_* spi_driver);								//!> Get SPI lines mode. Only RX or TX/RX Duplex
		uint8_t (*spi_get_data_size)(struct spi_driver_* spi_driver);								//!> Get SPI data size, from 4 to 16 bits
		spi_polarity_t (*spi_get_polarity)(struct spi_driver_* spi_driver);							//!> Get SPI polarity, high or low
		spi_phase_t (*spi_get_phase)(struct spi_driver_* spi_driver);								//!> Get SPI phase, 1 edge or 2 edge
		spi_nss_mode_t (*spi_get_nss_mode)(struct spi_driver_* spi_driver);							//!> Get SPI NSS signal mode, HW or SW
		uint32_t (*spi_get_speed)(struct spi_driver_* spi_driver);									//!> Get SPI bus speed
		uint32_t (*spi_get_timeout)(struct spi_driver_* spi_driver);								//!> Get SPI operations timeout
		spi_driver_state_t (*spi_driver_get_state)(struct spi_driver_* spi_driver);					//!> Returns the current state of the SPI driver
	}spi_config_funcs;
#endif
}spi_driver_t;


spi_driver_t* new_spi_driver(uint8_t spi_hw_number);
retval_t delete_spi_driver(spi_driver_t* spi_driver);

#endif /* APPLICATION_USER_BSP_YETIMOTE_PERIPHERAL_OS_SPI_H_ */
