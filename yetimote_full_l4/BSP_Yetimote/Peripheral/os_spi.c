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
 * os_spi.c
 *
 *  Created on: 16 de ene. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file os_spi.c
 */

#include "os_spi.h"

osMutexDef (os_spi_mutex);

#ifdef OS_SPI_DRIVER_FUNCS
extern spi_funcs_t OS_SPI_DRIVER_FUNCS;
#else
#warning	"SPI FUNC DRIVER NOT DEFINED"
#endif

#ifdef SPI_DRIVER_CONFIG_FUNCS
extern spi_config_funcs_t SPI_DRIVER_CONFIG_FUNCS;
#endif



spi_driver_t* new_spi_driver(uint8_t spi_hw_number){
	if(spi_hw_number >16){		//MAX NUMBER OF HW SPI INSTANCES 16
		return NULL;
	}
	spi_driver_t* new_spi_driver = (spi_driver_t*) pvPortMalloc(sizeof(spi_driver_t));
	new_spi_driver->spi_vars = (spi_vars_t*) pvPortMalloc(sizeof(spi_vars_t));
	new_spi_driver->spi_vars->spi_hw_number = spi_hw_number;
	new_spi_driver->spi_vars->opened_ports = NULL;
	new_spi_driver->spi_vars->driver_state = DRIVER_NOT_INIT;
	new_spi_driver->spi_vars->spi_mutex_id = osMutexCreate(osMutex(os_spi_mutex));

	new_spi_driver->spi_funcs = OS_SPI_DRIVER_FUNCS;
#if SPI_DRIVER_CONFIG_FUNCS
	new_spi_driver->spi_config_funcs = SPI_DRIVER_CONFIG_FUNCS;
#endif

	return new_spi_driver;
}
retval_t delete_spi_driver(spi_driver_t* spi_driver){

	if(spi_driver == NULL){
		return RET_ERROR;
	}
	osMutexWait(spi_driver->spi_vars->spi_mutex_id, osWaitForever);		//Driver must not be busy to delete it
	while(spi_driver->spi_vars->driver_state == DRIVER_BUSY);

	spi_driver->spi_vars->opened_ports = NULL;
	spi_driver->spi_vars->driver_state = DRIVER_NOT_INIT;
	spi_driver->spi_vars->spi_hw_number = 255;

	osMutexRelease(spi_driver->spi_vars->spi_mutex_id);

	osMutexDelete(spi_driver->spi_vars->spi_mutex_id);
	vPortFree(spi_driver->spi_vars);
	spi_driver->spi_vars = NULL;
	vPortFree(spi_driver);

	return RET_OK;
}
