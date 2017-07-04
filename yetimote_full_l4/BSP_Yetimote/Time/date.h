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
 * date.h
 *
 *  Created on: 4/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file date.h
 */

#ifndef APPLICATION_USER_BSP_YETIMOTE_TIME_DATE_H_
#define APPLICATION_USER_BSP_YETIMOTE_TIME_DATE_H_

#include "yetimote-conf.h"
#include "rtc.h"

typedef struct date_{
#if USE_RTC
	RTC_TimeTypeDef rtc_time;
	RTC_DateTypeDef rtc_date;
#else
	uint16_t second;
	uint16_t minute;
	uint16_t hour;
	uint16_t week_day;
	uint16_t month_day;
	uint16_t month;
	uint16_t year;
#endif
	uint16_t is_leap_year;
	osMutexId date_mutex_id;
}date_t;

date_t* new_date(uint64_t init_timestamp);
retval_t remove_date(date_t* date);

retval_t set_date(date_t* date, uint64_t timestamp);

uint16_t get_second(date_t* date);
uint16_t get_minute(date_t* date);
uint16_t get_hour(date_t* date);
uint16_t get_week_day(date_t* date);
uint16_t get_month_day(date_t* date);
uint16_t get_month(date_t* date);
uint16_t get_year(date_t* date);

retval_t set_second(date_t* date, uint16_t second);
retval_t set_minute(date_t* date, uint16_t minute);
retval_t set_hour(date_t* date, uint16_t hour);
retval_t set_week_day(date_t* date, uint16_t week_day);
retval_t set_month_day(date_t* date, uint16_t month_day);
retval_t set_month(date_t* date, uint16_t month);
retval_t set_year(date_t* date, uint16_t year);

#endif /* APPLICATION_USER_BSP_YETIMOTE_TIME_DATE_H_ */
