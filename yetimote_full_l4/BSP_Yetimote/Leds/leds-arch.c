/*
 * Copyright (c) 2010, Mariano Alvira <mar@devl.org> and other contributors
 * to the MC1322x project (http://mc1322x.devl.org) and Contiki.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki OS.
 *
 *
 */

#include "yetimote-conf.h"
#include "leds.h"
#include "gpio.h"

void leds_arch_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* set gpio func_sel to gpio (3) */
	  __GPIOB_CLK_ENABLE();
	  __GPIOA_CLK_ENABLE();
	  /*Configure GPIO pins : PB4 PB5 */
	  GPIO_InitStruct.Pin = LEDS_BLUE_PIN|LEDS_GREEN_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(LEDS_PORT_B, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA8 */
	  GPIO_InitStruct.Pin = LEDS_RED1_PIN|LEDS_RED2_PIN;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(LEDS_PORT_A, &GPIO_InitStruct);
}

unsigned char leds_arch_get(void)
{

	return((HAL_GPIO_ReadPin(LEDS_PORT_B, LEDS_BLUE_PIN) ? LEDS_BLUE : 0)
			|(HAL_GPIO_ReadPin(LEDS_PORT_B, LEDS_GREEN_PIN) ? LEDS_GREEN: 0)
			|(HAL_GPIO_ReadPin(LEDS_PORT_A, LEDS_RED1_PIN) ? LEDS_RED1: 0)
			|(HAL_GPIO_ReadPin(LEDS_PORT_A, LEDS_RED2_PIN) ? LEDS_RED2: 0));

}

void leds_arch_set(unsigned char leds)
{

	if(leds & LEDS_GREEN) { HAL_GPIO_WritePin(LEDS_PORT_B, LEDS_GREEN_PIN, GPIO_PIN_SET); } else {  HAL_GPIO_WritePin(LEDS_PORT_B, LEDS_GREEN_PIN, GPIO_PIN_RESET);}
	if(leds & LEDS_BLUE)  { HAL_GPIO_WritePin(LEDS_PORT_B, LEDS_BLUE_PIN, GPIO_PIN_SET); } else {  HAL_GPIO_WritePin(LEDS_PORT_B, LEDS_BLUE_PIN, GPIO_PIN_RESET);}
	if(leds & LEDS_RED1)  { HAL_GPIO_WritePin(LEDS_PORT_A, LEDS_RED1_PIN, GPIO_PIN_SET); } else {  HAL_GPIO_WritePin(LEDS_PORT_A, LEDS_RED1_PIN, GPIO_PIN_RESET);}
	if(leds & LEDS_RED2)  { HAL_GPIO_WritePin(LEDS_PORT_A, LEDS_RED2_PIN, GPIO_PIN_SET); } else {  HAL_GPIO_WritePin(LEDS_PORT_A, LEDS_RED2_PIN, GPIO_PIN_RESET);}
}

