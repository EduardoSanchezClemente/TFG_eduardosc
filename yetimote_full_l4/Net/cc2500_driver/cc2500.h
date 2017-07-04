/*
 * Copyright (c) 2013, Marcus Linderoth, http://forfunandprof.it
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         CC2500 driver header file
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#ifndef __CC2500_H__
#define __CC2500_H__


#include "spi.h"
#include "cc2500-const.h"

#include "timeFuncs.h"
#include "yetimote-conf.h"
#include "packetbuf.h"
#include "radio.h"

/* the length of rssi, checksum etc bytes appended by radio to packet */
#define FOOTER_LEN        2   // after the packet, two bytes RSSI+LQI+CRC are appended

#define RF_CHANNEL              0
/*
 * let the radio do destination address filtering in hardware; will conserve
 * power when used with SimpleRDC as unicasts will be ACKed immediately, letting
 * the sender go to sleep and we can use shorter wake-up periods on all nodes.
 */
#define USE_HW_ADDRESS_FILTER     	0
#if 	!USE_HW_ADDRESS_FILTER
#define USE_HW_ADDRESS_FILTER_LOW   0
#endif /* USE_HW_ADDRESS_FILTER */
#define USE_WOR 					0


/*---------------------------------------------------------------------------*/
/* useful macros */
/*
 * only OOK needs more than one field in PATABLE, the others are fine w one so
 * this works for all but OOK.
 */
#define CC2500_SET_TXPOWER(x)       cc2500_write_single(CC2500_PATABLE, x)

/* flush the Rx-/Tx-FIFOs */
#define FLUSH_FIFOS()     do {                                          \
                            cc2500_strobe(CC2500_SIDLE);                \
                            cc2500_strobe(CC2500_SFTX);                 \
                            cc2500_strobe(CC2500_SFRX);                 \
                          } while(0);

#define CC2500_STATUS()   ((cc2500_strobe(CC2500_SNOP) & CC2500_STATUSBYTE_STATUSBITS))


#define CC2500_MAX_PACKET_LEN       64

/* length of an ACK = address + seq# */
#define ACK_LEN   3



/* setting/clearing chip select help */
#define CC2500_SPI_ENABLE()         HAL_GPIO_WritePin(CC2500_SPI_CS_PORT , CC2500_SPI_CS_PIN , RESET);
#define CC2500_SPI_DISABLE()        HAL_GPIO_WritePin(CC2500_SPI_CS_PORT , CC2500_SPI_CS_PIN , SET);


#define CC2500_DEFAULT_CONFIG_LEN    47



extern const struct radio_driver cc2500_driver;

 struct radio_config_cc2500 {		//BLS
 	  /** Set WOR event0 period. */
 	  int (* set_WORevent0)(uint16_t event0);

 	  /** Set RX timeout for WOR. */
 	  int (* set_RXtimeout)(uint8_t rx_timeout);

 	  /** Enable RC Oscilator Calibration (Idle to RX). */
 	  int (* enable_RCOSC_CAL)(void);

 	  /** Disable RC Oscilator Calibration (Timed calibration only). */
 	  int (* disable_RCOSC_CAL)(void);
 };
 extern const struct radio_config cc2500_config;
/*--------------------------------------------------------------------------*/

int     cc2500_init(void);
void    cc2500_reset(void);
int     cc2500_interrupt(void);
int     cc2500_on(void);
int     cc2500_off(void);
int     cc2500_send(const void *payload, unsigned short payload_len);

int cc2500_rssi(void);
void    cc2500_set_channel(uint8_t c);
uint8_t cc2500_strobe(uint8_t strobe);
uint8_t cc2500_read_single(uint8_t adr);
uint8_t cc2500_read_burst(uint8_t adr, uint8_t *dest, uint8_t len);
uint8_t cc2500_write_single(uint8_t adr, uint8_t data);
uint8_t cc2500_write_burst(uint8_t adr, uint8_t *src, uint8_t len);

#endif /* __CC2500_H__ */

