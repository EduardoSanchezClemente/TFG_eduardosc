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
 * test_precesses.c
 *
 *  Created on: 21/10/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file test_processes.c
 */

#include "stm32l4xx.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "test_apps.h"

extern uint32_t avg_delay[TASKS_NUMBER];
extern uint32_t num_counts[TASKS_NUMBER];
extern uint32_t max_delay[TASKS_NUMBER];
extern uint32_t min_delay[TASKS_NUMBER];

extern uint32_t delayed[TASKS_NUMBER];

//TASK SET
void t1Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 1;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 19;
#endif
	uint32_t period = 5000;
	uint32_t rand_start = rand()%1000;

	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);

	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[0] += delay;
			num_counts[0]++;
			if(delay > max_delay[0]){
				max_delay[0] = delay;
			}
			if(delay < min_delay[0]){
				min_delay[0] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[0] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[0] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t2Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 4;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 22;
#endif
	uint32_t period = 4300;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[1] += delay;
			num_counts[1]++;
			if(delay > max_delay[1]){
				max_delay[1] = delay;
			}
			if(delay < min_delay[1]){
				min_delay[1] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[1] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[1] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t3Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 1;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 12;
#endif
	uint32_t period = 1080;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[2] += delay;
			num_counts[2]++;
			if(delay > max_delay[2]){
				max_delay[2] = delay;
			}
			if(delay < min_delay[2]){
				min_delay[2] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[2] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[2] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t4Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 5;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 25;
#endif
	uint32_t period = 504;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[3] += delay;
			num_counts[3]++;
			if(delay > max_delay[3]){
				max_delay[3] = delay;
			}
			if(delay < min_delay[3]){
				min_delay[3] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[3] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[3] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t5Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 10;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 19;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 55;
#endif
	uint32_t period = 5522;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[4] += delay;
			num_counts[4]++;
			if(delay > max_delay[4]){
				max_delay[4] = delay;
			}
			if(delay < min_delay[4]){
				min_delay[4] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[4] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[4] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t6Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 3;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 5;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 27;
#endif
	uint32_t period = 750;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[5] += delay;
			num_counts[5]++;
			if(delay > max_delay[5]){
				max_delay[5] = delay;
			}
			if(delay < min_delay[5]){
				min_delay[5] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[5] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[5] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t7Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 4;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 19;
#endif
	uint32_t period = 2515;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[6] += delay;
			num_counts[6]++;
			if(delay > max_delay[6]){
				max_delay[6] = delay;
			}
			if(delay < min_delay[6]){
				min_delay[6] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[6] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[6] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t8Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 7;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 11;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 33;
#endif
	uint32_t period = 3300;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[7] += delay;
			num_counts[7]++;
			if(delay > max_delay[7]){
				max_delay[7] = delay;
			}
			if(delay < min_delay[7]){
				min_delay[7] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[7] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[7] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t9Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 1;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 19;
#endif
	uint32_t period = 5000;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[8] += delay;
			num_counts[8]++;
			if(delay > max_delay[8]){
				max_delay[8] = delay;
			}
			if(delay < min_delay[8]){
				min_delay[8] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[8] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[8] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t10Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 4;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 22;
#endif
	uint32_t period = 4300;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[9] += delay;
			num_counts[9]++;
			if(delay > max_delay[9]){
				max_delay[9] = delay;
			}
			if(delay < min_delay[9]){
				min_delay[9] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[9] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[9] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t11Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 1;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 12;
#endif
	uint32_t period = 1080;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[10] += delay;
			num_counts[10]++;
			if(delay > max_delay[10]){
				max_delay[10] = delay;
			}
			if(delay < min_delay[10]){
				min_delay[10] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[10] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[10] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t12Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 5;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 25;
#endif
	uint32_t period = 504;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[11] += delay;
			num_counts[11]++;
			if(delay > max_delay[11]){
				max_delay[11] = delay;
			}
			if(delay < min_delay[11]){
				min_delay[11] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[11] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[11] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t13Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 10;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 19;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 55;
#endif
	uint32_t period = 5522;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[12] += delay;
			num_counts[12]++;
			if(delay > max_delay[12]){
				max_delay[12] = delay;
			}
			if(delay < min_delay[12]){
				min_delay[12] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[12] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[12] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t14Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 3;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 5;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 27;
#endif
	uint32_t period = 750;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[13] += delay;
			num_counts[13]++;
			if(delay > max_delay[13]){
				max_delay[13] = delay;
			}
			if(delay < min_delay[13]){
				min_delay[13] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[13] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[13] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t15Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 2;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 4;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 19;
#endif
	uint32_t period = 2515;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[14] += delay;
			num_counts[14]++;
			if(delay > max_delay[14]){
				max_delay[14] = delay;
			}
			if(delay < min_delay[14]){
				min_delay[14] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[14] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[14] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
void t16Task(void const * argument){
	uint32_t tick;
#if LOW_CPU_USAGE
	uint32_t expected_time = 7;
#endif
#if MED_CPU_USAGE
	uint32_t expected_time = 11;
#endif
#if HIGH_CPU_USAGE
	uint32_t expected_time = 33;
#endif
	uint32_t period = 3300;
	uint32_t rand_start = rand()%1000;
	osThreadSuspend(osThreadGetId());
//	osDelay(rand_start);
	while(1){
		uint32_t delay = 0;
		tick = osKernelSysTick();
		HAL_Delay(expected_time);
		tick = osKernelSysTick() - tick;
		tick = (tick * 1000) /configTICK_RATE_HZ;	//Parse to ms
		if(tick>=expected_time){
			delay = tick-expected_time;
		}
		else{
			delay = 0;
		}
		if (delay > 100000){
			delay = 0;
		}

		else{
			taskENTER_CRITICAL();
			avg_delay[15] += delay;
			num_counts[15]++;
			if(delay > max_delay[15]){
				max_delay[15] = delay;
			}
			if(delay < min_delay[15]){
				min_delay[15] = delay;
			}
			taskEXIT_CRITICAL();
		}


		if(tick <= period){
			taskENTER_CRITICAL();
			delayed[15] = 1;
			taskEXIT_CRITICAL();
			osDelay(period-tick);
			taskENTER_CRITICAL();
			delayed[15] = 0;
			taskEXIT_CRITICAL();
		}
	}
}
