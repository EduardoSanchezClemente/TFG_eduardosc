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
 * time_control.h
 *
 *  Created on: 4/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file time_control.h
 */

#ifndef APPLICATION_USER_BSP_YETIMOTE_TIME_TIME_CONTROL_H_
#define APPLICATION_USER_BSP_YETIMOTE_TIME_TIME_CONTROL_H_

#include "yetimote-conf.h"
#include "date.h"

typedef enum time_tick_state_{			//The time count could be stopped to reduce computation on systick ISR. By default it is running
	TIME_RUNNING,
	TIME_STOPPED,
}time_tick_state_t;

typedef struct time_control_{
	uint64_t timestamp;					//TIMESTAMP IN MS
	uint16_t prev_tick;
	date_t* date;
	osMutexId time_mutex_id;
	time_tick_state_t time_tick_state;
}time_control_t;


time_control_t* new_time_control(uint64_t init_timestamp);
retval_t remove_time_control(time_control_t* time_control);

retval_t update_time(time_control_t* time_control);			//Called from user domain. This function should be called ONLY when tick state is STOPPED

retval_t time_tick_callback(time_control_t* time_control);		//Called from Systick ISR or another time source ISR when tick state is RUNNING

retval_t start_tick_time_control(time_control_t* time_control);
retval_t stop_tick_time_control(time_control_t* time_control);

uint64_t get_timestamp(time_control_t* time_control);
retval_t set_timestamp(time_control_t* time_control, uint64_t timestamp);


#endif /* APPLICATION_USER_BSP_YETIMOTE_TIME_TIME_CONTROL_H_ */
