/*
 * Copyright (c) 2015, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * vcom_usb_inout.c
 *
 *  Created on: 2/3/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 */

#include "vcom_usb_inout.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/**
 * @brief								Writes the string in the USB output. The string MUST be \0 terminated
 * @param[in]		str					String to print using USB
 * @return								The number of bytes written
 */
uint16_t UsbWriteString(char* str){

	PCD_HandleTypeDef *hpcd = &hpcd_USB_OTG_FS;
	USBD_HandleTypeDef  *pdev = hpcd->pData;
	USBD_CDC_HandleTypeDef   *hcdc = (USBD_CDC_HandleTypeDef*) pdev->pClassData;

	uint16_t len = (uint16_t) strlen(str);
#if DEBUG_USB
	BUSYWAIT_UNTIL((hcdc->TxState != 1), SYSTEM_CONF_SECOND/50);

	if(pdev->dev_state != USB_STATE_SUSPENDED){	//De este modo nos aseguramos que no se envia nada si el cable USB no está conectado
		CDC_Transmit_FS((uint8_t*)str, len);					//Si no entraria en infinite loop al usar un printf
//		while((hcdc->TxState == 1));
	}
	else{
		len = -1;
	}
#endif

	return len;

}
