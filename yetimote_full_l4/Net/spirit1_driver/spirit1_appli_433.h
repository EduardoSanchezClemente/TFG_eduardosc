/**
  ******************************************************************************
  * @file    spirit1_appli.h 
  * @author  Central Labs
  * @version V1.1.0
  * @date    14-Aug-2014
  * @brief   Header for spirit1_appli.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPIRIT1_APPLI_433_H
#define __SPIRIT1_APPLI_433_H

/* Includes ------------------------------------------------------------------*/
//#include "cube_hal.h"
#include "stm32l4xx.h"
#include "MCU_Interface.h" 
#include "SPIRIT_Config.h"


/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/*  Radio configuration parameters  */
#define XTAL_OFFSET_PPM_433             0
#define INFINITE_TIMEOUT_433            0.0


#define BASE_FREQUENCY_433              433.05e6


#define CHANNEL_SPACE_433               100e3
#define CHANNEL_NUMBER_433              0
#define MODULATION_SELECT_433           GFSK_BT1
#define DATARATE_433                    38400
#define FREQ_DEVIATION_433              20e3
#define BANDWIDTH_433                   100E3
#define POWER_DBM_433                   11
#define POWER_INDEX_433                 0
#define RECEIVE_TIMEOUT_433             2000.0 /*change the value for required timeout period*/

#define RSSI_THRESHOLD_433              -120



#endif /* __SPIRIT1_APPLI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
