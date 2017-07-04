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
 * test_apps.h
 *
 *  Created on: 26/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file test_apps.h
 */

#ifndef APPLICATION_TEST_APPS_TEST_APPS_H_
#define APPLICATION_TEST_APPS_TEST_APPS_H_

//SPI COMMANDS
#define START_GLOBAL_TEST		0x01UL
#define START_NEXT				0x02UL
#define TIMEOUT					0x03UL
#define INTERRUPT_TEST			0x04UL
#define END_TEST				0x05UL

//INPUTS y PESOS
#define NUM_INPUTS				4

#define BAT  					0       // Porcentaje de bateria
#define RSSI  					1       // Porcentaje de nivel de señal en canal (respecto al máximo)
#define NET  					2       // Porcentaje de uso de cpu en los nodos vecinos
#define MULT_FACTOR  			3    	// Factor de multiplicación para la ganancia
#define PRIORITY  				4       // Prioridad de la tarea. Solo usado con los pesos ya que como input se pasa desde la lista de tareas
/* --------*/

#define COMMAND_SIZE			5

#define TASKS_NUMBER 			16

#define LOW_CPU_USAGE			0
#define MED_CPU_USAGE			1
#define HIGH_CPU_USAGE			0

#if LOW_CPU_USAGE
#define VERY_LOW_PRIORITY			osPriorityLow
#define BELOW_NORMAL_PRIORITY		osPriorityBelowNormal
#define NORMAL_PRIORITY				osPriorityNormal
#define ABOVE_NORMAL_PRIORITY		osPriorityAboveNormal
#define	HIGH_PRIORITY				osPriorityHigh
#define REAL_TIME_PRIORITY			osPriorityRealtime
#endif

#if MED_CPU_USAGE
#define VERY_LOW_PRIORITY			osPriorityBelowNormal
#define BELOW_NORMAL_PRIORITY		osPriorityBelowNormal
#define NORMAL_PRIORITY				osPriorityNormal
#define ABOVE_NORMAL_PRIORITY		osPriorityHigh
#define	HIGH_PRIORITY				osPriorityHigh
#define REAL_TIME_PRIORITY			osPriorityRealtime
#endif

#if HIGH_CPU_USAGE
#define VERY_LOW_PRIORITY			osPriorityBelowNormal
#define BELOW_NORMAL_PRIORITY		osPriorityNormal
#define NORMAL_PRIORITY				osPriorityNormal
#define ABOVE_NORMAL_PRIORITY		osPriorityHigh
#define	HIGH_PRIORITY				osPriorityRealtime
#define REAL_TIME_PRIORITY			osPriorityRealtime
#endif

#endif /* APPLICATION_TEST_APPS_TEST_APPS_H_ */
