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
 * os_i2c.c
 *
 *  Created on: 6 de feb. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file os_i2c.c
 */

#include "os_i2c.h"

osMutexDef (os_i2c_mutex);

#ifdef OS_I2C_DRIVER_FUNCS
extern i2c_funcs_t OS_I2C_DRIVER_FUNCS;
#else
#warning	"I2C FUNC DRIVER NOT DEFINED"
#endif

i2c_driver_t* new_i2c_driver(uint8_t i2c_hw_number){
	if(i2c_hw_number >16){		//MAX NUMBER OF HW I2C INSTANCES 16
		return NULL;
	}
	i2c_driver_t* new_i2c_driver = (i2c_driver_t*) pvPortMalloc(sizeof(i2c_driver_t));
	new_i2c_driver->i2c_vars = (i2c_vars_t*) pvPortMalloc(sizeof(i2c_vars_t));
	new_i2c_driver->i2c_vars->i2c_hw_number = i2c_hw_number;
	new_i2c_driver->i2c_vars->opened_ports = NULL;
	new_i2c_driver->i2c_vars->driver_state = DRIVER_NOT_INIT;
	new_i2c_driver->i2c_vars->i2c_mutex_id = osMutexCreate(osMutex(os_i2c_mutex));

	new_i2c_driver->i2c_funcs = OS_I2C_DRIVER_FUNCS;

	return new_i2c_driver;
}
retval_t delete_i2c_driver(i2c_driver_t* i2c_driver){

	if(i2c_driver == NULL){
		return RET_ERROR;
	}
	osMutexWait(i2c_driver->i2c_vars->i2c_mutex_id, osWaitForever);		//Driver must not be busy to delete it
	while(i2c_driver->i2c_vars->driver_state == DRIVER_BUSY);

	i2c_driver->i2c_vars->opened_ports = NULL;
	i2c_driver->i2c_vars->driver_state = DRIVER_NOT_INIT;
	i2c_driver->i2c_vars->i2c_hw_number = 255;

	osMutexRelease(i2c_driver->i2c_vars->i2c_mutex_id);

	osMutexDelete(i2c_driver->i2c_vars->i2c_mutex_id);
	vPortFree(i2c_driver->i2c_vars);
	i2c_driver->i2c_vars = NULL;
	vPortFree(i2c_driver);

	return RET_OK;
}
