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
 * time_control.c
 *
 *  Created on: 4/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file time_control.c
 */

#include "time_control.h"

osMutexDef (time_mutex);

time_control_t* new_time_control(uint64_t init_timestamp){
	time_control_t* new_time_control = (time_control_t*) pvPortMalloc(sizeof(time_control_t));

	new_time_control->timestamp = init_timestamp;

	new_time_control->date = new_date(init_timestamp);

#if USE_RTC
	HAL_RTC_GetTime(&hrtc, &new_time_control->date->rtc_time, RTC_FORMAT_BIN);
	new_time_control->prev_tick = new_time_control->date->rtc_time.SubSeconds;
#else
	new_time_control->prev_tick = osKernelSysTick();
#endif
	new_time_control->time_tick_state = TIME_RUNNING;

	new_time_control->time_mutex_id = osMutexCreate (osMutex (time_mutex));
	return new_time_control;
}

retval_t remove_time_control(time_control_t* time_control){

	remove_date(time_control->date);

	osMutexDelete (time_control->time_mutex_id);

	vPortFree(time_control);
	time_control = NULL;

	return RET_OK;
}


retval_t update_time(time_control_t* time_control){		//This function only works without RTC. With RTC is not necessary
#if !USE_RTC
	uint32_t current_tick = osKernelSysTick();
	osMutexWait(time_control->time_mutex_id, osWaitForever);
	time_control->timestamp += (uint64_t) (osKernelSysTick()- time_control->prev_tick)*portTICK_PERIOD_MS;		//No funciona con systicks mayores a 1000HZ
	time_control->prev_tick = current_tick;

	set_date(time_control->date, time_control->timestamp);

	osMutexRelease(time_control->time_mutex_id);
	return RET_OK;
#endif
	return RET_OK;
}

retval_t start_tick_time_control(time_control_t* time_control){
	osMutexWait(time_control->time_mutex_id, osWaitForever);
	time_control->time_tick_state = TIME_RUNNING;
	osMutexRelease(time_control->time_mutex_id);
	return RET_OK;
}

retval_t stop_tick_time_control(time_control_t* time_control){
	osMutexWait(time_control->time_mutex_id, osWaitForever);
	time_control->time_tick_state = TIME_STOPPED;
	osMutexRelease(time_control->time_mutex_id);
	return RET_OK;
}

uint64_t get_timestamp(time_control_t* time_control){
	osMutexWait(time_control->time_mutex_id, osWaitForever);
	uint64_t timestamp = time_control->timestamp;
	osMutexRelease(time_control->time_mutex_id);
	return timestamp;
}
retval_t set_timestamp(time_control_t* time_control, uint64_t timestamp){
	osMutexWait(time_control->time_mutex_id, osWaitForever);
	time_control->timestamp = timestamp;
	set_date(time_control->date, timestamp);
	osMutexRelease(time_control->time_mutex_id);
	return RET_OK;
}

retval_t time_tick_callback(time_control_t* time_control){
#if USE_RTC
	if(time_control->time_tick_state == TIME_RUNNING){
		HAL_RTC_GetTime(&hrtc, &time_control->date->rtc_time, RTC_FORMAT_BIN);
		uint16_t tick = time_control->date->rtc_time.SubSeconds;				//Subseconds counts from 999 to 0
		uint16_t elapsed_time = 0;
		if(tick > time_control->prev_tick){
			elapsed_time = time_control->prev_tick + 1000 - tick;
		}
		else {
			elapsed_time = time_control->prev_tick-tick;
		}
		time_control->prev_tick = tick;
		time_control->timestamp += elapsed_time;
	}
#else
	if(time_control->time_tick_state == TIME_RUNNING){
		uint32_t current_tick = osKernelSysTick();
		time_control->timestamp++;

		if(time_control->timestamp%1000 == 0){		//New second

			time_control->date->second++;

			if((time_control->date->second%60) == 0){	//New minute
				time_control->date->second = 0;
				time_control->date->minute++;

				if((time_control->date->minute%60) == 0){	//New hour
					time_control->date->minute = 0;
					time_control->date->hour++;

					if((time_control->date->hour%24) == 0){	//New Day
						time_control->date->hour = 0;
						time_control->date->week_day++;

						if((time_control->date->week_day%8) == 0){	//New week
							time_control->date->week_day = 1;
						}

						time_control->date->month_day++;
						switch(time_control->date->month){
						case 1:
							if(time_control->date->month_day > 31){	//New month
								time_control->date->month = 2;
								time_control->date->month_day = 1;
							}
							break;
						case 2:
							if(time_control->date->month_day > 28+time_control->date->is_leap_year){	//New month
								time_control->date->month = 3;
								time_control->date->month_day = 1;
							}
							break;
						case 3:
							if(time_control->date->month_day > 31){	//New month
								time_control->date->month = 4;
								time_control->date->month_day = 1;
							}
							break;
						case 4:
							if(time_control->date->month_day > 30){	//New month
								time_control->date->month = 5;
								time_control->date->month_day = 1;
							}
							break;
						case 5:
							if(time_control->date->month_day > 31){	//New month
								time_control->date->month = 6;
								time_control->date->month_day = 1;
							}
							break;
						case 6:
							if(time_control->date->month_day > 30){	//New month
								time_control->date->month = 7;
								time_control->date->month_day = 1;
							}
							break;
						case 7:
							if(time_control->date->month_day > 31){	//New month
								time_control->date->month = 8;
								time_control->date->month_day = 1;
							}
							break;
						case 8:
							if(time_control->date->month_day > 31){	//New month
								time_control->date->month = 9;
								time_control->date->month_day = 1;
							}
							break;
						case 9:
							if(time_control->date->month_day > 30){	//New month
								time_control->date->month = 10;
								time_control->date->month_day = 1;
							}
							break;
						case 10:
							if(time_control->date->month_day > 31){	//New month
								time_control->date->month = 11;
								time_control->date->month_day = 1;
							}
							break;
						case 11:
							if(time_control->date->month_day > 30){	//New month
								time_control->date->month = 12;
								time_control->date->month_day = 1;
							}
							break;
						case 12:
							if(time_control->date->month_day > 31){	//New year
								time_control->date->month = 1;
								time_control->date->month_day = 1;
								time_control->date->year++;
							}
							break;
						default:
							break;
						}

					}
				}

			}
		}
	}
#endif
	return RET_OK;
}


