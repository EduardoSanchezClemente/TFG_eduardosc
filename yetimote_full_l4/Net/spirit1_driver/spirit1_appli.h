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
#ifndef __SPIRIT1_APPLI_H
#define __SPIRIT1_APPLI_H

/* Includes ------------------------------------------------------------------*/
//#include "cube_hal.h"
#include "stm32l4xx.h"
#include "MCU_Interface.h" 
#include "SPIRIT_Config.h"


/************************Set the Node address**********************************/ 
/*Source address of one node should be destination for other node & vice-versa*/  


#define MY_ADDRESS                  0x44
#define DESTINATION_ADDRESS         0x44 

/******************************************************************************/
    


/**************************Uncomment the system Operating mode*****************/
//#define USE_LOW_POWER_MODE

#if defined (USE_LOW_POWER_MODE)
//#define MCU_SLEEP_MODE
#define MCU_STOP_MODE
#endif
/******************************************************************************/



/*************************Uncomment the the protocol to be used****************/
//#define USE_STack_PROTOCOL
#define USE_BASIC_PROTOCOL


#if defined(USE_STack_PROTOCOL)
//#define USE_STack_LLP  /*Uncomment if LLP featured need to be used*/
#endif
/******************************************************************************/


#if defined(MCU_SLEEP_MODE) || defined(MCU_STOP_MODE)
#define RF_STANDBY
#endif


/* Exported constants --------------------------------------------------------*/


/*  Packet configuration parameters  */
#define PREAMBLE_LENGTH             PKT_PREAMBLE_LENGTH_04BYTES
#define SYNC_LENGTH                 PKT_SYNC_LENGTH_4BYTES
#define SYNC_WORD                   0x88888888
#define LENGTH_TYPE                 PKT_LENGTH_VAR
#define LENGTH_WIDTH                12
#define CRC_MODE                    PKT_CRC_MODE_8BITS
#define CONTROL_LENGTH              PKT_CONTROL_LENGTH_0BYTES
#define EN_FEC                      S_ENABLE
#define EN_WHITENING                S_ENABLE

/*  Addresses configuration parameters  */
#define EN_ADDRESS                  S_DISABLE
#define EN_FILT_MY_ADDRESS          S_DISABLE
#define EN_FILT_MULTICAST_ADDRESS   S_DISABLE
#define EN_FILT_BROADCAST_ADDRESS   S_DISABLE
#define EN_FILT_SOURCE_ADDRESS      S_DISABLE//S_ENABLE
#define SOURCE_ADDR_MASK            0xf0
#define SOURCE_ADDR_REF             0x37
#define MULTICAST_ADDRESS           0xEE
#define BROADCAST_ADDRESS           0xFF 
    
    
#define EN_AUTOACK                      S_DISABLE
#define EN_PIGGYBACKING             	S_DISABLE
#define MAX_RETRANSMISSIONS         	PKT_DISABLE_RETX

#define TX_FIFO_ALMOST_EMPTY_BYTES		16
#define RX_FIFO_ALMOST_FULL_BYTES		48

/*  FIFO configuration parameters  */

/* Exported types ------------------------------------------------------------*/
extern volatile FlagStatus xRxDoneFlag, xTxDoneFlag;
extern volatile FlagStatus PushButtonStatusWakeup; 
extern uint16_t wakeupCounter;
extern uint16_t dataSendCounter ;
extern volatile FlagStatus PushButtonStatusData, datasendFlag;


/**
* @brief  WMBus Link Layer State Enum.
*/
typedef enum {
   SM_STATE_START_RX=0,
   SM_STATE_WAIT_FOR_RX_DONE,
   SM_STATE_DATA_RECEIVED,
   SM_STATE_SEND_DATA,
   SM_STATE_WAIT_FOR_TX_DONE,
   SM_STATE_ACK_RECEIVED,
   SM_STATE_SEND_ACK,
   SM_STATE_TOGGLE_LED,
   SM_STATE_IDLE=0xFF
} SM_State_t;

typedef struct sRadioDriver
{
    void ( *Init )( void );
    void ( *GpioIrq )( SGpioInit *pGpioIRQ );
    void ( *RadioInit )( SRadioInit *pRadioInit );
    void ( *SetRadioPower )( uint8_t cIndex, float fPowerdBm );
    void ( *PacketConfig )( void );
    void ( *SetPayloadLen )( uint8_t length);
    void ( *SetDestinationAddress )( uint8_t address);
    void ( *EnableTxIrq )( void );
    void ( *EnableRxIrq )( void );
    void ( *DisableIrq )(void);
    void ( *SetRxTimeout )( float cRxTimeout );
    void ( *EnableSQI )(void);
    void ( *SetRssiThreshold)(int dbmValue);
    void ( *ClearIrqStatus )(void);
    void ( *StartRx )( void );
    void ( *StartTx )( uint8_t *buffer, uint8_t size ); 
    void ( *GetRxPacket )( uint8_t *buffer, uint8_t *size );
}RadioDriver_t;   

typedef struct sMCULowPowerMode
{
    void ( *McuStopMode )( void );
    void ( *McuStandbyMode )( void );
    void ( *McuSleepMode )( void );      
}MCULowPowerMode_t;

typedef struct sRadioLowPowerMode
{
    void ( *RadioShutDown )( void );
    void ( *RadioStandBy )( void );
    void ( *RadioSleep ) ( void );
    void ( *RadioPowerON )( void );
}RadioLowPowerMode_t; 

typedef struct
{
  uint8_t Cmdtag;
  uint8_t CmdType;
  uint8_t CmdLen;
  uint8_t Cmd;
  uint8_t DataLen;
  uint8_t* DataBuff;
}AppliFrame_t;


/* Exported functions ------------------------------------------------------- */
void HAL_Spirit1_Init(void);
void P2P_Process(uint8_t *pTxBuff, uint8_t cTxlen, uint8_t* pRxBuff, uint8_t cRxlen);
void Enter_LP_mode(void);
void Exit_LP_mode(void);
void MCU_Enter_StopMode(void);
void MCU_Enter_StandbyMode(void);
void MCU_Enter_SleepMode(void);
void RadioPowerON(void);
void RadioPowerOFF(void);
void RadioStandBy(void);
void RadioSleep(void);
void AppliSendBuff(AppliFrame_t *xTxFrame, uint8_t cTxlen);
void AppliReceiveBuff(uint8_t *RxFrameBuff, uint8_t cRxlen);
void P2P_Init(void);
void STackProtocolInit(void);
void BasicProtocolInit(void);
void P2PInterruptHandler(void);
void Set_KeyStatus(FlagStatus val);


#endif /* __SPIRIT1_APPLI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
