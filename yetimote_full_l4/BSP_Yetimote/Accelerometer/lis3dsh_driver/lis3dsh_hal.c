/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
* File Name          : LIS3DSH_driver.c
* Author             : MSH Application Team
* Author             : Fabio Tota
* Version            : $Revision:$
* Date               : $Date:$
* Description        : LIS3DSH driver file
*                      
* HISTORY:
* Date               |	Modification                    |	Author
* 24/06/2011         |	Initial Revision                |	Fabio Tota
* 11/06/2012         |	Support for multiple drivers in the same program |	Abhishek Anand

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

/* Includes ------------------------------------------------------------------*/
#include "lis3dsh_hal.h"
#include "os_i2c.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern i2c_driver_t* i2c1_driver;
extern int16_t lis3dsh_i2c_port_id;
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* Function Name		: LIS3DSH_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*			: I2C or SPI reading functions					
* Input			: Register Address
* Output		: Data REad
* Return		: None
*******************************************************************************/
u8_t LIS3DSH_ReadReg(u8_t Reg, u8_t* Data) {
	port_state_t i2c_state = i2c1_driver->i2c_funcs.i2c_port_get_state(i2c1_driver, lis3dsh_i2c_port_id);
	if(i2c_state != DRIVER_PORT_CLOSED){
		i2c1_driver->i2c_funcs.i2c_read_reg(i2c1_driver, lis3dsh_i2c_port_id, Reg, 1,(uint8_t*) Data, 1);
	}
//  HAL_I2C_Mem_Read(&hi2c1, LIS3DSH_MEMS_I2C_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, Data, 1, 500);
  //To be completed with either I2c or SPI reading function
  //i.e. *Data = SPI_Mems_Read_Reg( Reg );  
  return 1;
}


/*******************************************************************************
* Function Name		: LIS3DSH_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*			: I2C or SPI writing function
* Input			: Register Address, Data to be written
* Output		: None
* Return		: None
*******************************************************************************/
u8_t LIS3DSH_WriteReg(u8_t WriteAddr, u8_t Data) {

	port_state_t i2c_state = i2c1_driver->i2c_funcs.i2c_port_get_state(i2c1_driver, lis3dsh_i2c_port_id);
	if(i2c_state != DRIVER_PORT_CLOSED){
		i2c1_driver->i2c_funcs.i2c_write_reg(i2c1_driver, lis3dsh_i2c_port_id, WriteAddr, 1,(uint8_t*) &Data, 1);
	}
//  HAL_I2C_Mem_Write(&hi2c1, LIS3DSH_MEMS_I2C_ADDRESS, WriteAddr, 1, &Data, 1, 500);
  //To be completed with either I2c or SPI writing function
  //i.e. SPI_Mems_Write_Reg(WriteAddr, Data);  
  return 1;
}


/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : LIS3DSH_GetWHO_AM_I
* Description    : Read identification code by WHO_AM_I register
* Input          : Char to empty by Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
//status_t LIS3DSH_GetWHO_AM_I(u8_t* val){
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_WHO_AM_I, val) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetStatusAUX
* Description    : Read the AUX status register
* Input          : Char to empty by status register buffer
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetStatusAUX(u8_t* val) {
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_STATUS_AUX, val) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}



/*******************************************************************************
* Function Name  : LIS3DSH_GetStatusAUXBIT
* Description    : Read the AUX status register BIT
* Input          : LIS3DSH_STATUS_AUX_321OR, LIS3DSH_STATUS_AUX_3OR, LIS3DSH_STATUS_AUX_2OR, LIS3DSH_STATUS_AUX_1OR,
                   LIS3DSH_STATUS_AUX_321DA, LIS3DSH_STATUS_AUX_3DA, LIS3DSH_STATUS_AUX_2DA, LIS3DSH_STATUS_AUX_1DA
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetStatusAUXBit(u8_t statusBIT, u8_t* val) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_STATUS_AUX, &value) )
//    return MEMS_ERROR;
//
//  if(statusBIT == LIS3DSH_STATUS_AUX_321OR){
//    if(value &= LIS3DSH_STATUS_AUX_321OR){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_STATUS_AUX_3OR){
//    if(value &= LIS3DSH_STATUS_AUX_3OR){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_STATUS_AUX_2OR){
//    if(value &= LIS3DSH_STATUS_AUX_2OR){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_STATUS_AUX_1OR){
//    if(value &= LIS3DSH_STATUS_AUX_1OR){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_STATUS_AUX_321DA){
//    if(value &= LIS3DSH_STATUS_AUX_321DA) {
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_STATUS_AUX_3DA){
//    if(value &= LIS3DSH_STATUS_AUX_3DA){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_STATUS_AUX_2DA){
//    if(value &= LIS3DSH_STATUS_AUX_2DA){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_STATUS_AUX_1DA){
//    if(value &= LIS3DSH_STATUS_AUX_1DA){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//  return MEMS_ERROR;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetODR
* Description    : Sets LIS3DSH Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_SetODR(LIS3DSH_ODR_t ov){
  u8_t value;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0x0f;
  value |= ov<<LIS3DSH_ODR_BIT;
  
  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DSH_SetTemperature
* Description    : Sets LIS3DSH Output Temperature
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Note           : For Read Temperature by LIS3DSH_OUT_AUX_3, LIS3DSH_SetADCAux and LIS3DSH_SetBDU
				   functions must be ENABLE
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetTemperature(State_t state){
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_TEMP_CFG_REG, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xBF;
//  value |= state<<LIS3DSH_TEMP_EN;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_TEMP_CFG_REG, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetADCAux
* Description    : Sets LIS3DSH Output ADC
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetADCAux(State_t state){
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_TEMP_CFG_REG, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x7F;
//  value |= state<<LIS3DSH_ADC_PD;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_TEMP_CFG_REG, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetAuxRaw
* Description    : Read the Aux Values Output Registers
* Input          : Buffer to empty
* Output         : Aux Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetAuxRaw(LIS3DSH_Aux123Raw_t* buff) {
//  u8_t valueL;
//  u8_t valueH;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_1_L, &valueL) )
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_1_H, &valueH) )
//    return MEMS_ERROR;
//
//  buff->AUX_1 = (u16_t)( (valueH << 8) | valueL )/16;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_2_L, &valueL) )
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_2_H, &valueH) )
//    return MEMS_ERROR;
//
//  buff->AUX_2 = (u16_t)( (valueH << 8) | valueL )/16;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_3_L, &valueL) )
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_3_H, &valueH) )
//    return MEMS_ERROR;
//
//  buff->AUX_3 = (u16_t)( (valueH << 8) | valueL )/16;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetTempRaw
* Description    : Read the Temperature Values by AUX Output Registers OUT_3_H
* Input          : Buffer to empty
* Output         : Temperature Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetTempRaw(i8_t* buff) {
//  u8_t valueL;
//  u8_t valueH;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_3_L, &valueL) )
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_3_H, &valueH) )
//    return MEMS_ERROR;
//
//  *buff = (i8_t)( valueH );
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetMode
* Description    : Sets LIS3DSH Operating Mode
* Input          : Modality (LIS3DSH_NORMAL, LIS3DSH_POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_SetMode(LIS3DSH_Mode_t md) {
	  u8_t value;

	  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG4, &value) )
	    return MEMS_ERROR;

	  value &= 0x0f;
	  value |= md<<LIS3DSH_ODR_BIT;

	  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4, value) )
	    return MEMS_ERROR;

	  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DSH_SetAxis
* Description    : Enable/Disable LIS3DSH Axis
* Input          : LIS3DSH_X_ENABLE/DISABLE | LIS3DSH_Y_ENABLE/DISABLE | LIS3DSH_Z_ENABLE/DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_SetAxis(LIS3DSH_Axis_t axis) {
  u8_t value;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG4, &value) )
    return MEMS_ERROR;
  value &= 0xF8;
  value |= (0x07 & axis);
  
  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4, value) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DSH_SetFullScale
* Description    : Sets the LIS3DSH FullScale
* Input          : LIS3DSH_FULLSCALE_2/LIS3DSH_FULLSCALE_4/LIS3DSH_FULLSCALE_8/LIS3DSH_FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_SetFullScale(LIS3DSH_Fullscale_t fs) {
  u8_t value;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG5, &value) )
    return MEMS_ERROR;
  
  value &= 0xC7;
  value |= (fs<<LIS3DSH_FSCALE);
  
  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DSH_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetBDU(State_t bdu) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG4, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x7F;
//  value |= (bdu<<LIS3DSH_BDU);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : BLE_LSB / BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetBLE(LIS3DSH_Endianess_t ble) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG4, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xBF;
//  value |= (ble<<LIS3DSH_BLE);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetSelfTest
* Description    : Set Self Test Modality
* Input          : LIS3DSH_SELF_TEST_DISABLE/ST_0/ST_1
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetSelfTest(LIS3DSH_SelfTest_t st) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG4, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xF9;
//  value |= (st<<LIS3DSH_ST);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_HPFClick
* Description    : Enable/Disable High Pass Filter for click
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_HPFClickEnable(State_t hpfe) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG2, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xFB;
//  value |= (hpfe<<LIS3DSH_HPCLICK);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG2, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_HPFAOI1
* Description    : Enable/Disable High Pass Filter for AOI on INT_1
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_HPFAOI1Enable(State_t hpfe) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG2, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xFE;
//  value |= (hpfe<<LIS3DSH_HPIS1);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG2, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_HPFAOI2
* Description    : Enable/Disable High Pass Filter for AOI on INT_2
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_HPFAOI2Enable(State_t hpfe) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG2, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xFD;
//  value |= (hpfe<<LIS3DSH_HPIS2);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG2, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : LIS3DSH_HPM_NORMAL_MODE_RES/LIS3DSH_HPM_REF_SIGNAL/
				   LIS3DSH_HPM_NORMAL_MODE/LIS3DSH_HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetHPFMode(LIS3DSH_HPFMode_t hpm) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG2, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x3F;
//  value |= (hpm<<LIS3DSH_HPM);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG2, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetHPFCutOFF(LIS3DSH_HPFCutOffFreq_t hpf) {
//  u8_t value;
//
//  if (hpf > 3)
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG2, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xCF;
//  value |= (hpf<<LIS3DSH_HPCF);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG2, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetFilterDataSel
* Description    : Set Filter Data Selection bypassed or sent to FIFO OUT register
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetFilterDataSel(State_t state) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG2, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xF7;
//  value |= (state<<LIS3DSH_FDS);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG2, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          :  LIS3DSH_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | LIS3DSH_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |
                    LIS3DSH_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | LIS3DSH_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |
                    LIS3DSH_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | LIS3DSH_WTM_ON_INT1_ENABLE/DISABLE         |
                    LIS3DSH_INT1_OVERRUN_ENABLE/DISABLE
* example        : SetInt1Pin(LIS3DSH_CLICK_ON_PIN_INT1_ENABLE | LIS3DSH_I1_INT1_ON_PIN_INT1_ENABLE |
                    LIS3DSH_I1_INT2_ON_PIN_INT1_DISABLE | LIS3DSH_I1_DRDY1_ON_INT1_ENABLE | LIS3DSH_I1_DRDY2_ON_INT1_ENABLE |
                    LIS3DSH_WTM_ON_INT1_DISABLE | LIS3DSH_INT1_OVERRUN_DISABLE   )
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetInt1Pin(LIS3DSH_IntPinConf_t pinConf) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG3, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x00;
//  value |= pinConf;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG3, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : LIS3DSH_CLICK_ON_PIN_INT2_ENABLE/DISABLE   | LIS3DSH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |
                   LIS3DSH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS3DSH_I2_BOOT_ON_INT2_ENABLE/DISABLE |
                   LIS3DSH_INT_ACTIVE_HIGH/LOW
* example        : LIS3DSH_SetInt2Pin(LIS3DSH_CLICK_ON_PIN_INT2_ENABLE/DISABLE | LIS3DSH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |
                   LIS3DSH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS3DSH_I2_BOOT_ON_INT2_ENABLE/DISABLE |
                   LIS3DSH_INT_ACTIVE_HIGH/LOW)
* Note           : To enable Interrupt signals on INT2 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetInt2Pin(LIS3DSH_IntPinConf_t pinConf) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG6, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x00;
//  value |= pinConf;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG6, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetClickCFG
* Description    : Set Click Interrupt config Function
* Input          : LIS3DSH_ZD_ENABLE/DISABLE | LIS3DSH_ZS_ENABLE/DISABLE  | LIS3DSH_YD_ENABLE/DISABLE  |
                   LIS3DSH_YS_ENABLE/DISABLE | LIS3DSH_XD_ENABLE/DISABLE  | LIS3DSH_XS_ENABLE/DISABLE
* example        : LIS3DSH_SetClickCFG( LIS3DSH_ZD_ENABLE | LIS3DSH_ZS_DISABLE | LIS3DSH_YD_ENABLE |
                               LIS3DSH_YS_DISABLE | LIS3DSH_XD_ENABLE | LIS3DSH_XS_ENABLE)
* Note           : You MUST use all input variable in the argument, as example
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetClickCFG(u8_t status) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CLICK_CFG, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xC0;
//  value |= status;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CLICK_CFG, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetClickTHS
* Description    : Set Click Interrupt threshold
* Input          : Click-click Threshold value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetClickTHS(u8_t ths) {
//
//  if(ths>127)
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CLICK_THS, ths) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetClickLIMIT
* Description    : Set Click Interrupt Time Limit
* Input          : Click-click Time Limit value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetClickLIMIT(u8_t t_limit) {
//
//  if(t_limit>127)
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_TIME_LIMIT, t_limit) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetClickLATENCY
* Description    : Set Click Interrupt Time Latency
* Input          : Click-click Time Latency value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetClickLATENCY(u8_t t_latency) {
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_TIME_LATENCY, t_latency) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetClickWINDOW
* Description    : Set Click Interrupt Time Window
* Input          : Click-click Time Window value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetClickWINDOW(u8_t t_window) {
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_TIME_WINDOW, t_window) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetClickResponse
* Description    : Get Click Interrupt Response by CLICK_SRC REGISTER
* Input          : char to empty by Click Response Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetClickResponse(u8_t* res) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CLICK_SRC, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x7F;
//
//  if((value & LIS3DSH_IA)==0) {
//    *res = LIS3DSH_NO_CLICK;
//    return MEMS_SUCCESS;
//  }
//  else {
//    if (value & LIS3DSH_DCLICK){
//      if (value & LIS3DSH_CLICK_SIGN){
//        if (value & LIS3DSH_CLICK_Z) {
//          *res = LIS3DSH_DCLICK_Z_N;
//          return MEMS_SUCCESS;
//        }
//        if (value & LIS3DSH_CLICK_Y) {
//          *res = LIS3DSH_DCLICK_Y_N;
//          return MEMS_SUCCESS;
//        }
//        if (value & LIS3DSH_CLICK_X) {
//          *res = LIS3DSH_DCLICK_X_N;
//          return MEMS_SUCCESS;
//        }
//      }
//      else{
//        if (value & LIS3DSH_CLICK_Z) {
//          *res = LIS3DSH_DCLICK_Z_P;
//          return MEMS_SUCCESS;
//        }
//        if (value & LIS3DSH_CLICK_Y) {
//          *res = LIS3DSH_DCLICK_Y_P;
//          return MEMS_SUCCESS;
//        }
//        if (value & LIS3DSH_CLICK_X) {
//          *res = LIS3DSH_DCLICK_X_P;
//          return MEMS_SUCCESS;
//        }
//      }
//    }
//    else{
//      if (value & LIS3DSH_CLICK_SIGN){
//        if (value & LIS3DSH_CLICK_Z) {
//          *res = LIS3DSH_SCLICK_Z_N;
//          return MEMS_SUCCESS;
//        }
//        if (value & LIS3DSH_CLICK_Y) {
//          *res = LIS3DSH_SCLICK_Y_N;
//          return MEMS_SUCCESS;
//        }
//        if (value & LIS3DSH_CLICK_X) {
//          *res = LIS3DSH_SCLICK_X_N;
//          return MEMS_SUCCESS;
//        }
//      }
//      else{
//        if (value & LIS3DSH_CLICK_Z) {
//          *res = LIS3DSH_SCLICK_Z_P;
//          return MEMS_SUCCESS;
//        }
//        if (value & LIS3DSH_CLICK_Y) {
//          *res = LIS3DSH_SCLICK_Y_P;
//          return MEMS_SUCCESS;
//        }
//        if (value & LIS3DSH_CLICK_X) {
//          *res = LIS3DSH_SCLICK_X_P;
//          return MEMS_SUCCESS;
//        }
//      }
//    }
//  }
//  return MEMS_ERROR;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_Int1LatchEnable(State_t latch) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG5, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xF7;
//  value |= latch<<LIS3DSH_LIR_INT1;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG5, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_ResetInt1Latch(void) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_INT1_SRC, &value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetIntConfiguration
* Description    : Interrupt 1 Configuration (without LIS3DSH_6D_INT)
* Input          : LIS3DSH_INT1_AND/OR | LIS3DSH_INT1_ZHIE_ENABLE/DISABLE | LIS3DSH_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetIntConfiguration(LIS3DSH_Int1Conf_t ic) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_INT1_CFG, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x40;
//  value |= ic;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_INT1_CFG, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}

     
/*******************************************************************************
* Function Name  : LIS3DSH_SetIntMode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : LIS3DSH_INT_MODE_OR, LIS3DSH_INT_MODE_6D_MOVEMENT, LIS3DSH_INT_MODE_AND,
				   LIS3DSH_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetIntMode(LIS3DSH_Int1Mode_t int_mode) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_INT1_CFG, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x3F;
//  value |= (int_mode<<LIS3DSH_INT_6D);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_INT1_CFG, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}

    
/*******************************************************************************
* Function Name  : LIS3DSH_SetInt6D4DConfiguration
* Description    : 6D, 4D Interrupt Configuration
* Input          : LIS3DSH_INT1_6D_ENABLE, LIS3DSH_INT1_4D_ENABLE, LIS3DSH_INT1_6D_4D_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetInt6D4DConfiguration(LIS3DSH_INT_6D_4D_t ic) {
//  u8_t value;
//  u8_t value2;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_INT1_CFG, &value) )
//    return MEMS_ERROR;
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG5, &value2) )
//    return MEMS_ERROR;
//
//  if(ic == LIS3DSH_INT1_6D_ENABLE){
//    value &= 0xBF;
//    value |= (MEMS_ENABLE<<LIS3DSH_INT_6D);
//    value2 &= 0xFB;
//    value2 |= (MEMS_DISABLE<<LIS3DSH_D4D_INT1);
//  }
//
//  if(ic == LIS3DSH_INT1_4D_ENABLE){
//    value &= 0xBF;
//    value |= (MEMS_ENABLE<<LIS3DSH_INT_6D);
//    value2 &= 0xFB;
//    value2 |= (MEMS_ENABLE<<LIS3DSH_D4D_INT1);
//  }
//
//  if(ic == LIS3DSH_INT1_6D_4D_DISABLE){
//    value &= 0xBF;
//    value |= (MEMS_DISABLE<<LIS3DSH_INT_6D);
//    value2 &= 0xFB;
//    value2 |= (MEMS_DISABLE<<LIS3DSH_D4D_INT1);
//  }
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_INT1_CFG, value) )
//    return MEMS_ERROR;
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG5, value2) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_Get6DPosition
* Description    : 6D, 4D Interrupt Position Detect
* Input          : Byte to empty by POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_Get6DPosition(u8_t* val){
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_INT1_SRC, &value) )
//    return MEMS_ERROR;
//
//  value &= 0x7F;
//
//  switch (value){
//  case LIS3DSH_UP_SX:
//    *val = LIS3DSH_UP_SX;
//    break;
//  case LIS3DSH_UP_DX:
//    *val = LIS3DSH_UP_DX;
//    break;
//  case LIS3DSH_DW_SX:
//    *val = LIS3DSH_DW_SX;
//    break;
//  case LIS3DSH_DW_DX:
//    *val = LIS3DSH_DW_DX;
//    break;
//  case LIS3DSH_TOP:
//    *val = LIS3DSH_TOP;
//    break;
//  case LIS3DSH_BOTTOM:
//    *val = LIS3DSH_BOTTOM;
//    break;
//  }
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetInt1Threshold(u8_t ths) {
//  if (ths > 127)
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_INT1_THS, ths) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetInt1Duration(LIS3DSH_Int1Conf_t id) {
//
//  if (id > 127)
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_INT1_DURATION, id) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : LIS3DSH_FIFO_DISABLE, LIS3DSH_FIFO_BYPASS_MODE, LIS3DSH_FIFO_MODE,
				   LIS3DSH_FIFO_STREAM_MODE, LIS3DSH_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_FIFOModeEnable(LIS3DSH_FifoMode_t fm) {
  u8_t value;  
  
  if(fm == LIS3DSH_FIFO_DISABLE) {
    if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1F;
    value |= (LIS3DSH_FIFO_BYPASS_MODE<<LIS3DSH_FM);
    
    if( !LIS3DSH_WriteReg(LIS3DSH_FIFO_CTRL_REG, value) )           //fifo mode bypass
      return MEMS_ERROR;   
    if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG6, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;    
    
    if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG6, value) )               //fifo disable
      return MEMS_ERROR;   
  }
  
  if(fm == LIS3DSH_FIFO_BYPASS_MODE)   {
    if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG6, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS3DSH_FIFO_EN;
    
    if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG6, value) )               //fifo enable
      return MEMS_ERROR;  
    if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS3DSH_FM);                     //fifo mode configuration
    
    if( !LIS3DSH_WriteReg(LIS3DSH_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS3DSH_FIFO_MODE)   {
    if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG6, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS3DSH_FIFO_EN;
    
    if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG6, value) )               //fifo enable
      return MEMS_ERROR;  
    if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS3DSH_FM);                      //fifo mode configuration
    
    if( !LIS3DSH_WriteReg(LIS3DSH_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS3DSH_FIFO_STREAM_MODE)   {
    if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG6, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS3DSH_FIFO_EN;
    
    if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG6, value) )               //fifo enable
      return MEMS_ERROR;   
    if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS3DSH_FM);                      //fifo mode configuration
    
    if( !LIS3DSH_WriteReg(LIS3DSH_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS3DSH_FIFO_TRIGGER_MODE)   {
    if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG6, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS3DSH_FIFO_EN;
    
    if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG6, value) )               //fifo enable
      return MEMS_ERROR;    
    if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS3DSH_FM);                      //fifo mode configuration
    
    if( !LIS3DSH_WriteReg(LIS3DSH_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DSH_SetTriggerInt
* Description    : Trigger event liked to trigger signal INT1/INT2
* Input          : LIS3DSH_TRIG_INT1/LIS3DSH_TRIG_INT2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetTriggerInt(LIS3DSH_TrigInt_t tr) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_CTRL_REG, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xDF;
//  value |= (tr<<LIS3DSH_TR);
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_FIFO_CTRL_REG, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetWaterMark(u8_t wtm) {
//  u8_t value;
//
//  if(wtm > 31)
//    return MEMS_ERROR;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_CTRL_REG, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xE0;
//  value |= wtm;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_FIFO_CTRL_REG, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}

  
/*******************************************************************************
* Function Name  : LIS3DSH_GetStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetStatusReg(u8_t* val) {
//  if( !LIS3DSH_ReadReg(LIS3DSH_STATUS_REG, val) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetStatusBIT
* Description    : Read the status register BIT
* Input          : LIS3DSH_STATUS_REG_ZYXOR, LIS3DSH_STATUS_REG_ZOR, LIS3DSH_STATUS_REG_YOR, LIS3DSH_STATUS_REG_XOR,
                   LIS3DSH_STATUS_REG_ZYXDA, LIS3DSH_STATUS_REG_ZDA, LIS3DSH_STATUS_REG_YDA, LIS3DSH_STATUS_REG_XDA,
				   LIS3DSH_DATAREADY_BIT
				   val: Byte to be filled with the status bit	
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetStatusBit(u8_t statusBIT, u8_t* val) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_STATUS_REG, &value) )
//    return MEMS_ERROR;
//
//  switch (statusBIT){
//  case LIS3DSH_STATUS_REG_ZYXOR:
//    if(value &= LIS3DSH_STATUS_REG_ZYXOR){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  case LIS3DSH_STATUS_REG_ZOR:
//    if(value &= LIS3DSH_STATUS_REG_ZOR){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  case LIS3DSH_STATUS_REG_YOR:
//    if(value &= LIS3DSH_STATUS_REG_YOR){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  case LIS3DSH_STATUS_REG_XOR:
//    if(value &= LIS3DSH_STATUS_REG_XOR){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  case LIS3DSH_STATUS_REG_ZYXDA:
//    if(value &= LIS3DSH_STATUS_REG_ZYXDA){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  case LIS3DSH_STATUS_REG_ZDA:
//    if(value &= LIS3DSH_STATUS_REG_ZDA){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  case LIS3DSH_STATUS_REG_YDA:
//    if(value &= LIS3DSH_STATUS_REG_YDA){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  case LIS3DSH_STATUS_REG_XDA:
//    if(value &= LIS3DSH_STATUS_REG_XDA){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//
//  }
//  return MEMS_ERROR;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DSH_GetAccAxesRaw(AxesRaw_t* buff) {
  i16_t value;
  u8_t *valueL = (u8_t *)(&value);
  u8_t *valueH = ((u8_t *)(&value)+1);
  
  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_X_L, valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_X_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_X = value;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_Y_L, valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_Y_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Y = value;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_Z_L, valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_OUT_Z_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Z = value;
  
  return MEMS_SUCCESS; 
}


/*******************************************************************************
* Function Name  : LIS3DSH_GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : Char to empty by Int1 source value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetInt1Src(u8_t* val) {
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_INT1_SRC, val) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : statusBIT: LIS3DSH_INT_SRC_IA, LIS3DSH_INT_SRC_ZH, LIS3DSH_INT_SRC_ZL.....
*                  val: Byte to be filled with the status bit
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetInt1SrcBit(u8_t statusBIT, u8_t* val) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_INT1_SRC, &value) )
//      return MEMS_ERROR;
//
//  if(statusBIT == LIS3DSH_INT1_SRC_IA){
//    if(value &= LIS3DSH_INT1_SRC_IA){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_INT1_SRC_ZH){
//    if(value &= LIS3DSH_INT1_SRC_ZH){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_INT1_SRC_ZL){
//    if(value &= LIS3DSH_INT1_SRC_ZL){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_INT1_SRC_YH){
//    if(value &= LIS3DSH_INT1_SRC_YH){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_INT1_SRC_YL){
//    if(value &= LIS3DSH_INT1_SRC_YL){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//  if(statusBIT == LIS3DSH_INT1_SRC_XH){
//    if(value &= LIS3DSH_INT1_SRC_XH){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_INT1_SRC_XL){
//    if(value &= LIS3DSH_INT1_SRC_XL){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//  return MEMS_ERROR;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : Byte to empty by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetFifoSourceReg(u8_t* val) {
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_SRC_REG, val) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetFifoSourceBit
* Description    : Read Fifo WaterMark source bit
* Input          : statusBIT: LIS3DSH_FIFO_SRC_WTM, LIS3DSH_FIFO_SRC_OVRUN, LIS3DSH_FIFO_SRC_EMPTY
*				   val: Byte to fill  with the bit value
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_GetFifoSourceBit(u8_t statusBIT,  u8_t* val){
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_SRC_REG, &value) )
//    return MEMS_ERROR;
//
//
//  if(statusBIT == LIS3DSH_FIFO_SRC_WTM){
//    if(value &= LIS3DSH_FIFO_SRC_WTM){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//
//  if(statusBIT == LIS3DSH_FIFO_SRC_OVRUN){
//    if(value &= LIS3DSH_FIFO_SRC_OVRUN){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//  if(statusBIT == LIS3DSH_FIFO_SRC_EMPTY){
//    if(value &= statusBIT == LIS3DSH_FIFO_SRC_EMPTY){
//      *val = MEMS_SET;
//      return MEMS_SUCCESS;
//    }
//    else{
//      *val = MEMS_RESET;
//      return MEMS_SUCCESS;
//    }
//  }
//  return MEMS_ERROR;
//}


/*******************************************************************************
* Function Name  : LIS3DSH_GetFifoSourceFSS
* Description    : Read current number of unread samples stored in FIFO
* Input          : Byte to empty by FIFO unread sample value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS3DSH_GetFifoSourceFSS(u8_t* val){
  u8_t value;
  
  if( !LIS3DSH_ReadReg(LIS3DSH_FIFO_SRC_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0x1F;
  
  *val = value;
  
  return MEMS_SUCCESS;
}

      
/*******************************************************************************
* Function Name  : LIS3DSH_SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : LIS3DSH_SPI_3_WIRE, LIS3DSH_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
//status_t LIS3DSH_SetSPIInterface(LIS3DSH_SPIMode_t spi) {
//  u8_t value;
//
//  if( !LIS3DSH_ReadReg(LIS3DSH_CTRL_REG4, &value) )
//    return MEMS_ERROR;
//
//  value &= 0xFE;
//  value |= spi<<LIS3DSH_SIM;
//
//  if( !LIS3DSH_WriteReg(LIS3DSH_CTRL_REG4, value) )
//    return MEMS_ERROR;
//
//  return MEMS_SUCCESS;
//}
/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
