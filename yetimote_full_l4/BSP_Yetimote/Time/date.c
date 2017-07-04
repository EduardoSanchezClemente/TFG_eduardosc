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
 * date.c
 *
 *  Created on: 4/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file date.c
 */


#include "date.h"

osMutexDef (date_mutex);

date_t* new_date(uint64_t init_timestamp){
	date_t* new_date = (date_t*) pvPortMalloc(sizeof(date_t));

	new_date->date_mutex_id = osMutexCreate (osMutex (date_mutex));

	set_date(new_date, init_timestamp);

	return new_date;
}

retval_t remove_date(date_t* date){

	osMutexDelete(date->date_mutex_id);

	vPortFree(date);
	date = NULL;

	return RET_OK;
}

#if USE_RTC
retval_t set_date(date_t* date, uint64_t timestamp){
	uint32_t day;
	uint16_t elapsed_leap_years = 0;
	uint16_t i;
	uint16_t real_year;
	uint16_t local_weekday;

	osMutexWait(date->date_mutex_id, osWaitForever);
	date->is_leap_year = 0;
	uint64_t year = timestamp/1000;
	year = year/365;
	year = year/24;
	year = year/3600;
	year = year + 1970;
	real_year = year;
	year = real_year-2000;
	date->rtc_date.Year = (uint8_t) year;

	for(i=1972; i<real_year; i+=4){		//1972 First leap year since 1970
		elapsed_leap_years++;
	}

	if((real_year%4) == 0){			//Gregorian calendar for leap years
		date->is_leap_year = 1;
		if((real_year%100) == 0){
			date->is_leap_year = 0;
		}
		if((real_year%400) == 0){
			date->is_leap_year = 1;
		}
	}
	day = (uint32_t)(timestamp/(1000*3600*24));			//The elapsed days since 1st January 1970

	local_weekday = (uint16_t) (day%7);					//Init day Thursday 1st January 1970
	if(local_weekday >= 4){
		date->rtc_date.WeekDay = local_weekday - 3;
	}
	else{
		date->rtc_date.WeekDay = local_weekday + 4;
	}

	day = day - ((real_year-1970)*365) - elapsed_leap_years + 1;			//The day on this year

	if(day<=31){
		date->rtc_date.Month = 1;
		date->rtc_date.Date = day;
	}
	else if(day<= (59+date->is_leap_year)){
		date->rtc_date.Month = 2;
		date->rtc_date.Date = day-31;
	}
	else if(day<= (90+date->is_leap_year)){
		date->rtc_date.Month = 3;
		date->rtc_date.Date = day-(59+date->is_leap_year);
	}
	else if(day<= (120+date->is_leap_year)){
		date->rtc_date.Month = 4;
		date->rtc_date.Date = day-(90+date->is_leap_year);
	}
	else if(day<= (151+date->is_leap_year)){
		date->rtc_date.Month = 5;
		date->rtc_date.Date = day-(120+date->is_leap_year);
	}
	else if(day<= (181+date->is_leap_year)){
		date->rtc_date.Month = 6;
		date->rtc_date.Date = day-(151+date->is_leap_year);
	}
	else if(day<= (212+date->is_leap_year)){
		date->rtc_date.Month = 7;
		date->rtc_date.Date = day-(181+date->is_leap_year);
	}
	else if(day<= (243+date->is_leap_year)){
		date->rtc_date.Month = 8;
		date->rtc_date.Date = day-(212+date->is_leap_year);
	}
	else if(day<= (273+date->is_leap_year)){
		date->rtc_date.Month = 9;
		date->rtc_date.Date = day-(243+date->is_leap_year);
	}
	else if(day<= (304+date->is_leap_year)){
		date->rtc_date.Month = 10;
		date->rtc_date.Date = day-(273+date->is_leap_year);
	}
	else if(day<= (334+date->is_leap_year)){
		date->rtc_date.Month = 11;
		date->rtc_date.Date = day-(304+date->is_leap_year);
	}
	else if(day<= (365+date->is_leap_year)){
		date->rtc_date.Month = 12;
		date->rtc_date.Date = day-(334+date->is_leap_year);
	}
	date->rtc_time.Hours = (timestamp/(1000*3600))%24;
	date->rtc_time.Minutes = (timestamp/(1000*60))%60;
	date->rtc_time.Seconds = (timestamp/(1000))%60;
	date->rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;
	date->rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	date->rtc_time.TimeFormat = RTC_FORMAT_BIN;

	HAL_RTC_SetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_SetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);

	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}

uint16_t get_second(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	HAL_RTC_GetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
	uint16_t second = date->rtc_time.Seconds;
	osMutexRelease(date->date_mutex_id);
	return second;
}

uint16_t get_minute(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	HAL_RTC_GetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
	uint16_t minute = date->rtc_time.Minutes;
	osMutexRelease(date->date_mutex_id);
	return minute;
}
uint16_t get_hour(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	HAL_RTC_GetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
	uint16_t hour = date->rtc_time.Hours;
	osMutexRelease(date->date_mutex_id);
	return hour;
}
uint16_t get_week_day(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	HAL_RTC_GetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
	uint16_t week_day = date->rtc_date.WeekDay;
	osMutexRelease(date->date_mutex_id);
	return week_day;
}
uint16_t get_month_day(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	HAL_RTC_GetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
	uint16_t month_day = date->rtc_date.Date;
	osMutexRelease(date->date_mutex_id);
	return month_day;
}
uint16_t get_month(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	HAL_RTC_GetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
	uint16_t month = date->rtc_date.Month;
	osMutexRelease(date->date_mutex_id);
	return month;
}
uint16_t get_year(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	HAL_RTC_GetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
	uint16_t year = date->rtc_date.Year + 2000;					//Correction. The year has only two digits in the RTC
	osMutexRelease(date->date_mutex_id);
	return year;
}


retval_t set_second(date_t* date, uint16_t second){
	if((second<60) && (second>=0)){
		osMutexWait(date->date_mutex_id, osWaitForever);
		HAL_RTC_GetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
		date->rtc_time.Seconds = second;
		date->rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;
		date->rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		date->rtc_time.TimeFormat = RTC_FORMAT_BIN;
		HAL_RTC_SetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
		osMutexRelease(date->date_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

retval_t set_minute(date_t* date, uint16_t minute){
	if((minute<60) && (minute>=0)){
		osMutexWait(date->date_mutex_id, osWaitForever);
		HAL_RTC_GetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
		date->rtc_time.Minutes = minute;
		date->rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;
		date->rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		date->rtc_time.TimeFormat = RTC_FORMAT_BIN;
		HAL_RTC_SetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
		osMutexRelease(date->date_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
retval_t set_hour(date_t* date, uint16_t hour){
	if((hour<24) && (hour>=0)){
		osMutexWait(date->date_mutex_id, osWaitForever);
		HAL_RTC_GetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
		date->rtc_time.Hours = hour;
		date->rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;
		date->rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		date->rtc_time.TimeFormat = RTC_FORMAT_BIN;
		HAL_RTC_SetTime(&hrtc, &date->rtc_time, RTC_FORMAT_BIN);
		osMutexRelease(date->date_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
retval_t set_week_day(date_t* date, uint16_t week_day){
	if((week_day<=7) && (week_day>0)){
		osMutexWait(date->date_mutex_id, osWaitForever);
		HAL_RTC_GetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
		date->rtc_date.WeekDay = week_day;
		HAL_RTC_SetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
		osMutexRelease(date->date_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
retval_t set_month_day(date_t* date, uint16_t month_day){
	if((month_day<=31) && (month_day>0)){
		osMutexWait(date->date_mutex_id, osWaitForever);
		HAL_RTC_GetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
		date->rtc_date.Date = month_day;
		HAL_RTC_SetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
		osMutexRelease(date->date_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
retval_t set_month(date_t* date, uint16_t month){
	if((month<=12) && (month>0)){
		osMutexWait(date->date_mutex_id, osWaitForever);
		HAL_RTC_GetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
		date->rtc_date.Month = month;
		HAL_RTC_SetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
		osMutexRelease(date->date_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}
retval_t set_year(date_t* date, uint16_t year){
	if((year<100) && (year>=0)){
		osMutexWait(date->date_mutex_id, osWaitForever);
		HAL_RTC_GetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
		date->rtc_date.Year = year;
		HAL_RTC_SetDate(&hrtc, &date->rtc_date, RTC_FORMAT_BIN);
		osMutexRelease(date->date_mutex_id);
		return RET_OK;
	}
	else{
		return RET_ERROR;
	}
}

#else
retval_t set_date(date_t* date, uint64_t timestamp){
	uint32_t day;
	uint16_t elapsed_leap_years = 0;
	uint16_t i;

	osMutexWait(date->date_mutex_id, osWaitForever);
	date->is_leap_year = 0;
	uint64_t year = timestamp/1000;
	year = year/365;
	year = year/24;
	year = year/3600;
	year = year + 1970;
	date->year = (uint16_t) year;

	for(i=1972; i<date->year; i+=4){		//1972 First leap year since 1970
		elapsed_leap_years++;
	}

	if((date->year%4) == 0){			//Gregorian calendar for leap years
		date->is_leap_year = 1;
		if((date->year%100) == 0){
			date->is_leap_year = 0;
		}
		if((date->year%400) == 0){
			date->is_leap_year = 1;
		}
	}
	day = (uint32_t)(timestamp/(1000*3600*24));			//The elapsed days since 1st January 1970

	date->week_day = (uint16_t) (day%7);					//Init day Thursday 1st January 1970
	if(date->week_day >= 4){
		date->week_day -= 3;
	}
	else{
		date->week_day += 4;
	}

	day = day - ((date->year-1970)*365) - elapsed_leap_years + 1;			//The day on this year

	if(day<=31){
		date->month = 1;
		date->month_day = day;
	}
	else if(day<= (59+date->is_leap_year)){
		date->month = 2;
		date->month_day = day-31;
	}
	else if(day<= (90+date->is_leap_year)){
		date->month = 3;
		date->month_day = day-(59+date->is_leap_year);
	}
	else if(day<= (120+date->is_leap_year)){
		date->month = 4;
		date->month_day = day-(90+date->is_leap_year);
	}
	else if(day<= (151+date->is_leap_year)){
		date->month = 5;
		date->month_day = day-(120+date->is_leap_year);
	}
	else if(day<= (181+date->is_leap_year)){
		date->month = 6;
		date->month_day = day-(151+date->is_leap_year);
	}
	else if(day<= (212+date->is_leap_year)){
		date->month = 7;
		date->month_day = day-(181+date->is_leap_year);
	}
	else if(day<= (243+date->is_leap_year)){
		date->month = 8;
		date->month_day = day-(212+date->is_leap_year);
	}
	else if(day<= (273+date->is_leap_year)){
		date->month = 9;
		date->month_day = day-(243+date->is_leap_year);
	}
	else if(day<= (304+date->is_leap_year)){
		date->month = 10;
		date->month_day = day-(273+date->is_leap_year);
	}
	else if(day<= (334+date->is_leap_year)){
		date->month = 11;
		date->month_day = day-(304+date->is_leap_year);
	}
	else if(day<= (365+date->is_leap_year)){
		date->month = 12;
		date->month_day = day-(334+date->is_leap_year);
	}

	date->hour = (uint16_t)(timestamp/(1000*3600))%24;
	date->minute = (uint16_t)(timestamp/(1000*60))%60;
	date->second = (uint16_t)(timestamp/(1000))%60;

	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}

uint16_t get_second(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	uint16_t second = date->second;
	osMutexRelease(date->date_mutex_id);
	return second;
}

uint16_t get_minute(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	uint16_t minute = date->minute;
	osMutexRelease(date->date_mutex_id);
	return minute;
}
uint16_t get_hour(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	uint16_t hour = date->hour;
	osMutexRelease(date->date_mutex_id);
	return hour;
}
uint16_t get_week_day(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	uint16_t week_day = date->week_day;
	osMutexRelease(date->date_mutex_id);
	return week_day;
}
uint16_t get_month_day(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	uint16_t month_day = date->month_day;
	osMutexRelease(date->date_mutex_id);
	return month_day;
}
uint16_t get_month(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	uint16_t month = date->month;
	osMutexRelease(date->date_mutex_id);
	return month;
}
uint16_t get_year(date_t* date){
	osMutexWait(date->date_mutex_id, osWaitForever);
	uint16_t year = date->year;
	osMutexRelease(date->date_mutex_id);
	return year;
}


retval_t set_second(date_t* date, uint16_t second){
	osMutexWait(date->date_mutex_id, osWaitForever);
	date->second = second;
	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}

retval_t set_minute(date_t* date, uint16_t minute){
	osMutexWait(date->date_mutex_id, osWaitForever);
	date->minute = minute;
	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}
retval_t set_hour(date_t* date, uint16_t hour){
	osMutexWait(date->date_mutex_id, osWaitForever);
	date->hour = hour;
	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}
retval_t set_week_day(date_t* date, uint16_t week_day){
	osMutexWait(date->date_mutex_id, osWaitForever);
	date->week_day = week_day;
	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}
retval_t set_month_day(date_t* date, uint16_t month_day){
	osMutexWait(date->date_mutex_id, osWaitForever);
	date->month_day = month_day;
	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}
retval_t set_month(date_t* date, uint16_t month){
	osMutexWait(date->date_mutex_id, osWaitForever);
	date->month = month;
	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}
retval_t set_year(date_t* date, uint16_t year){
	osMutexWait(date->date_mutex_id, osWaitForever);
	date->year = year;
	osMutexRelease(date->date_mutex_id);
	return RET_OK;
}

#endif
