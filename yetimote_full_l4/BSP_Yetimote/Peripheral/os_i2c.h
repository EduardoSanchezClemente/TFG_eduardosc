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
 * os_i2c.h
 *
 *  Created on: 6 de feb. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file os_i2c.h
 */
#ifndef APPLICATION_USER_BSP_YETIMOTE_PERIPHERAL_OS_I2C_H_
#define APPLICATION_USER_BSP_YETIMOTE_PERIPHERAL_OS_I2C_H_

#include "yetimote-conf.h"
#include "periph_drivers.h"
#include "generic_list.h"


typedef enum{
	I2C_PORT_SLAVE,				//!> I2C PORT slave mode
	I2C_PORT_MASTER,			//!> I2C PORT master mode
}i2c_port_mode_t;

/**
 * @brief	Struct containing a specific virtual port configuration variables for I2C
 */
typedef struct i2c_port_{
	int16_t port_id;		//!> Virtual port id opened
	uint8_t device_addr;	//!> Physical address of the i2c slave connected to this port
	i2c_port_mode_t i2c_port_mode;	//!> I2C port mode: MASTER or SLAVE
	port_state_t port_state;	//!> Current state of this i2c port
}i2c_port_t;

/**
 * @brief Struct containing the variables used by an SPI driver
 */
typedef struct i2c_vars_{
	uint8_t i2c_hw_number;			//!> Number of HW I2C instance

	uint8_t i2c_own_addr;			//!> Own Physical address of the i2c bus
	uint32_t default_i2c_timeout;	//!> SPI read write timeout. If 0 it means an infinite timeout

	gen_list* opened_ports;			//!> List with the ports opened. Each element in the list is a i2c_port_t type element.

	driver_state_t driver_state;	//!> Current state of the i2c driver
	osMutexId i2c_mutex_id;			//!> I2C driver mutex. Only one thread can use driver functions at a time
}i2c_vars_t;

typedef struct i2c_funcs_ i2c_funcs_t;

/**
 * @brief	I2C Driver object. It must be an object for each physical I2C bus.
 */
typedef struct i2c_driver_{
	i2c_vars_t* i2c_vars;						//!> I2C driver variables

	struct i2c_funcs_{
		retval_t (*i2c_init)(struct i2c_driver_* i2c_driver, uint8_t own_addr, uint32_t default_i2c_timeout);					//!> Initialize and configures a hardware I2C device. It must configure all HW parameters of the I2C
		retval_t (*i2c_deinit)(struct i2c_driver_* i2c_driver);																	//!> Disable an initialized hardware I2C device
		int16_t (*i2c_open)(struct i2c_driver_* i2c_driver, uint8_t device_addr, i2c_port_mode_t i2c_port_mode);						//!> Open a virtual port associated with an slave address. It returns the virtual port id opened (-1 if error)
		retval_t (*i2c_close)(struct i2c_driver_* i2c_driver, int16_t port_number);													//!> Close a previously opened virtual port
		retval_t (*i2c_read)(struct i2c_driver_* i2c_driver, int16_t port_number, uint8_t *prx, uint16_t size);						//!> Read from i2c the specified bytes number. It is a blocking function. It blocks till completed or i2c timeout reached
		retval_t (*i2c_read_nb)(struct i2c_driver_* i2c_driver, int16_t port_number, uint8_t *prx, uint16_t size);					//!> Read from i2c the specified bytes number in non blocking mode. The current thread sleeps till completed or i2c timeout reached
		retval_t (*i2c_write)(struct i2c_driver_* i2c_driver, int16_t port_number, uint8_t *ptx, uint16_t size);						//!> Write to i2c the specified bytes number. It is a blocking function. It blocks till completed or i2c timeout reached
		retval_t (*i2c_write_nb)(struct i2c_driver_* i2c_driver, int16_t port_number, uint8_t *ptx, uint16_t size);					//!> Write from i2c the specified bytes number in non blocking mode. The current thread sleeps till completed or i2c timeout reached
		retval_t (*i2c_read_reg)(struct i2c_driver_* i2c_driver, int16_t port_number, uint16_t reg_addr, uint16_t reg_addr_size,
				uint8_t *prx, uint16_t size);																				//!> Read from i2c on an internal register the specified bytes number. It blocks till completed or i2c timeout reached
		retval_t (*i2c_write_reg)(struct i2c_driver_* i2c_driver, int16_t port_number, uint16_t reg_addr, uint16_t reg_addr_size,
				uint8_t *ptx, uint16_t size);																				//!> Write to i2c on an internal register the specified bytes number. It blocks till completed or i2c timeout reached
		port_state_t (*i2c_port_get_state)(struct i2c_driver_* i2c_driver, int16_t port_number); 									//!> Returns the current state of the specified port
	}i2c_funcs;
}i2c_driver_t;

i2c_driver_t* new_i2c_driver(uint8_t i2c_hw_number);
retval_t delete_i2c_driver(i2c_driver_t* i2c_driver);

#endif /* APPLICATION_USER_BSP_YETIMOTE_PERIPHERAL_OS_I2C_H_ */
