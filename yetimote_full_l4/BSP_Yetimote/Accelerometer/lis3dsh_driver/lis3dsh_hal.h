/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : lis3dsh_driver.h
* Author             : MSH Application Team
* Author             : Abhishek Anand, Fabio Tota
* Version            : $Revision:$
* Date               : $Date:$
* Description        : Descriptor Header for lis3dsh_driver.c driver file
*
* HISTORY:
* Date        | Modification                                | Author
* 24/06/2011  | Initial Revision                            | Fabio Tota
* 07/06/2012  | Support for multiple drivers in the same program | Abhishek Anand
*
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIS3DSH_DRIVER__H
#define __LIS3DSH_DRIVER__H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/


//these could change accordingly with the architecture

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES

typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef short int i16_t;
typedef signed char i8_t;

#endif /*__ARCHDEP__TYPES*/

typedef u8_t LIS3DSH_IntPinConf_t;
typedef u8_t LIS3DSH_Axis_t;
typedef u8_t LIS3DSH_Int1Conf_t;


//define structure
#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef enum {
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00	
} status_t;

typedef enum {
  MEMS_ENABLE				=		0x01,
  MEMS_DISABLE				=		0x00	
} State_t;

typedef struct {
  i16_t AXIS_X;
  i16_t AXIS_Y;
  i16_t AXIS_Z;
} AxesRaw_t;

#endif /*__SHARED__TYPES*/

typedef enum {  
  LIS3DSH_ODR_3_125Hz		    =		0x01,
  LIS3DSH_ODR_6_25Hz            =		0x02,
  LIS3DSH_ODR_12_5Hz		    =		0x03,
  LIS3DSH_ODR_25Hz		        =		0x04,
  LIS3DSH_ODR_50Hz		        =		0x05,
  LIS3DSH_ODR_100Hz		        =		0x06,
  LIS3DSH_ODR_400Hz		        =		0x07,
  LIS3DSH_ODR_800Hz		        =		0x08,
  LIS3DSH_ODR_1600Hz			=		0x09
} LIS3DSH_ODR_t;

typedef enum {
  LIS3DSH_POWER_DOWN                =		0x00,
  LIS3DSH_NORMAL					=		0x07
} LIS3DSH_Mode_t;


typedef enum {
  LIS3DSH_FULLSCALE_2                   =               0x00,
  LIS3DSH_FULLSCALE_4                   =               0x01,
  LIS3DSH_FULLSCALE_8                   =               0x02,
  LIS3DSH_FULLSCALE_16                  =               0x03
} LIS3DSH_Fullscale_t;

typedef enum {
  LIS3DSH_FIFO_BYPASS_MODE              =               0x00,
  LIS3DSH_FIFO_MODE                     =               0x01,
  LIS3DSH_FIFO_STREAM_MODE              =               0x02,
  LIS3DSH_FIFO_TRIGGER_MODE             =               0x03,
  LIS3DSH_FIFO_DISABLE                  =               0x04
} LIS3DSH_FifoMode_t;

typedef enum {
  LIS3DSH_X_ENABLE                      =               0x01,
  LIS3DSH_X_DISABLE                     =               0x00,
  LIS3DSH_Y_ENABLE                      =               0x02,
  LIS3DSH_Y_DISABLE                     =               0x00,
  LIS3DSH_Z_ENABLE                      =               0x04,
  LIS3DSH_Z_DISABLE                     =               0x00
} LIS3DSH_AXISenable_t;

//TODO: start from here and manage the shared macros etc before this

/* Exported constants --------------------------------------------------------*/

#ifndef __SHARED__CONSTANTS
#define __SHARED__CONSTANTS

#define MEMS_SET                                        0x01
#define MEMS_RESET                                      0x00

#endif /*__SHARED__CONSTANTS*/




/* ******************NEW RRZ*****************/

//CONTROL REGISTER 4
#define LIS3DSH_CTRL_REG4				0x20
#define LIS3DSH_ODR_BIT				    BIT(4)
#define LIS3DSH_BDU						BIT(3)
#define LIS3DSH_ZEN						BIT(2)
#define LIS3DSH_YEN						BIT(1)
#define LIS3DSH_XEN						BIT(0)

//CONTROL REGISTER 5
#define LIS3DSH_CTRL_REG5				0x24
#define LIS3DSH_BW	                    BIT(6)
#define LIS3DSH_FSCALE                  BIT(3)
#define LIS3DSH_ST		                BIT(1)
#define LIS3DSH_SIM			            BIT(0)

//CONTROL REGISTER 6
#define LIS3DSH_CTRL_REG6				0x25
#define LIS3DSH_BOOT	                BIT(7)
#define LIS3DSH_FIFO_EN	                BIT(6)
#define LIS3DSH_WTM_EN	                BIT(5)
#define LIS3DSH_ADD_INC	                BIT(4)
#define LIS3DSH_P1_EMPTY                BIT(3)
#define LIS3DSH_P1_WTM	                BIT(2)
#define LIS3DSH_OVERRUN	                BIT(1)
#define LIS3DSH_P2_BOOT	                BIT(0)


//FIFO REGISTERS
//FIFO CONTROL REGISTER
#define LIS3DSH_FIFO_CTRL_REG                           0x2E
#define LIS3DSH_FM                                      BIT(5)
#define LIS3DSH_FTH                                     BIT(0)

//FIFO SRC REGISTER
#define LIS3DSH_FIFO_SRC_REG			        0x2F

//OUTPUT REGISTER
#define LIS3DSH_OUT_X_L					0x28
#define LIS3DSH_OUT_X_H					0x29
#define LIS3DSH_OUT_Y_L					0x2A
#define LIS3DSH_OUT_Y_H					0x2B
#define LIS3DSH_OUT_Z_L					0x2C
#define LIS3DSH_OUT_Z_H					0x2D

/* **************************************/

#define LIS3DSH_MEMS_I2C_ADDRESS			        0x3D


/* Exported macro ------------------------------------------------------------*/

#ifndef __SHARED__MACROS

#define __SHARED__MACROS
#define ValBit(VAR,Place)         (VAR & (1<<Place))
#define BIT(x) ( (x) )

#endif /*__SHARED__MACROS*/

/* Exported functions --------------------------------------------------------*/

u8_t LIS3DSH_ReadReg(u8_t Reg, u8_t* Data);
u8_t LIS3DSH_WriteReg(u8_t WriteAddr, u8_t Data);

//Sensor Configuration Functions
status_t LIS3DSH_SetODR(LIS3DSH_ODR_t ov);
status_t LIS3DSH_SetMode(LIS3DSH_Mode_t md);
status_t LIS3DSH_SetAxis(LIS3DSH_Axis_t axis);
status_t LIS3DSH_SetFullScale(LIS3DSH_Fullscale_t fs);

//Filtering Functions
//status_t LIS3DSH_HPFClickEnable(State_t hpfe);
//status_t LIS3DSH_HPFAOI1Enable(State_t hpfe);
//status_t LIS3DSH_HPFAOI2Enable(State_t hpfe);
//status_t LIS3DSH_SetHPFMode(LIS3DSH_HPFMode_t hpf);
//status_t LIS3DSH_SetHPFCutOFF(LIS3DSH_HPFCutOffFreq_t hpf);
//status_t LIS3DSH_SetFilterDataSel(State_t state);

//Interrupt Functions
//status_t LIS3DSH_SetInt1Pin(LIS3DSH_IntPinConf_t pinConf);
//status_t LIS3DSH_SetInt2Pin(LIS3DSH_IntPinConf_t pinConf);
//status_t LIS3DSH_Int1LatchEnable(State_t latch);
//status_t LIS3DSH_ResetInt1Latch(void);
//status_t LIS3DSH_SetIntConfiguration(LIS3DSH_Int1Conf_t ic);
//status_t LIS3DSH_SetInt1Threshold(u8_t ths);
//status_t LIS3DSH_SetInt1Duration(LIS3DSH_Int1Conf_t id);
//status_t LIS3DSH_SetIntMode(LIS3DSH_Int1Mode_t ic);
//status_t LIS3DSH_SetClickCFG(u8_t status);
//status_t LIS3DSH_SetInt6D4DConfiguration(LIS3DSH_INT_6D_4D_t ic);
//status_t LIS3DSH_GetInt1Src(u8_t* val);
//status_t LIS3DSH_GetInt1SrcBit(u8_t statusBIT, u8_t* val);

//FIFO Functions
status_t LIS3DSH_FIFOModeEnable(LIS3DSH_FifoMode_t fm);
//status_t LIS3DSH_SetWaterMark(u8_t wtm);
//status_t LIS3DSH_SetTriggerInt(LIS3DSH_TrigInt_t tr);
//status_t LIS3DSH_GetFifoSourceReg(u8_t* val);
//status_t LIS3DSH_GetFifoSourceBit(u8_t statusBIT, u8_t* val);
status_t LIS3DSH_GetFifoSourceFSS(u8_t* val);

//Other Reading Functions
//status_t LIS3DSH_GetStatusReg(u8_t* val);
//status_t LIS3DSH_GetStatusBit(u8_t statusBIT, u8_t* val);
//status_t LIS3DSH_GetStatusAUXBit(u8_t statusBIT, u8_t* val);
//status_t LIS3DSH_GetStatusAUX(u8_t* val);
status_t LIS3DSH_GetAccAxesRaw(AxesRaw_t* buff);
//status_t LIS3DSH_GetAuxRaw(LIS3DSH_Aux123Raw_t* buff);
//status_t LIS3DSH_GetClickResponse(u8_t* val);
//status_t LIS3DSH_GetTempRaw(i8_t* val);
//status_t LIS3DSH_GetWHO_AM_I(u8_t* val);
//status_t LIS3DSH_Get6DPosition(u8_t* val);

//Generic
// i.e. u8_t LIS3DSH_ReadReg(u8_t Reg, u8_t* Data);
// i.e. u8_t LIS3DSH_WriteReg(u8_t Reg, u8_t Data);


#endif /* __LIS3DSH_H */

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/



