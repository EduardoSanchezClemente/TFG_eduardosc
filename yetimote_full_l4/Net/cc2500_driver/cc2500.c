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
 *         CC2500 driver, intentionally simple and naive in order to save RAM/ROM
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include <string.h>
#include <stdio.h>
#include "spi.h"
#include "rime.h"
#include "netstack.h"
#include "cc2500.h"
#include "cc2500-const.h"
#include "cc2500-config.h"

/*---------------------------------------------------------------------------*/

#if USE_WOR
static uint8_t RSOSC_disabled = 0;
#endif /* USE_WOR */
#if USE_HW_ADDRESS_FILTER_LOW
static uint8_t hw_dest = 0;
static int test_count = 0;
#endif /* USE_HW_ADDRESS_FILTER_LOW */

void cc2500Task_function(void const * argument);

osThreadId cc2500TaskId = NULL;

/*---------------------------------------------------------------------------*/
/* function prototypes for the radio driver */
       int  cc2500_init(void);
static int  cc2500_prepare(const void *data, unsigned short len);
static int  cc2500_transmit(unsigned short len);
       int  cc2500_send(const void *data, unsigned short len);
static int  cc2500_read(void *buf, unsigned short bufsize);
static int  cc2500_channel_clear(void);
static int  cc2500_receiving_packet(void);
static int  cc2500_pending_packet(void);
       int  cc2500_on(void);
       int  cc2500_off(void);
       int  cc2500_close(void);
#if USE_WOR
       int  cc2500_set_WORevent0(uint16_t event0);
       int  cc2500_set_RXtimeout(uint8_t RX_timeout);
       int	cc2500_enable_RCOSC_CAL(void);
       int	cc2500_disable_RCOSC_CAL(void);
#endif /* USE_WOR */

extern const uint8_t cc2500_default_config[];

osMutexId cc2500_mutex;
osMutexDef(cc2500_mutex);

/* flags */
static uint8_t cc2500_is_on = 0;
static uint8_t cc2500_should_off = 0;


uint8_t cc2500_tx_power = 0x0E;	//-20dBm

/* define the radio driver */
const struct radio_driver cc2500_driver =
{
  /** init the radio */
  cc2500_init,

  /** Prepare the radio with a packet to be sent. */
  cc2500_prepare,

  /** Send the packet that has previously been prepared. */
  cc2500_transmit,

  /** Prepare & transmit a packet. */
  /* radio return values.
        RADIO_TX_OK,
        RADIO_TX_ERR,
        RADIO_TX_COLLISION,
        RADIO_TX_NOACK,
  */
  cc2500_send,

  /** Read a received packet into a buffer. */
  cc2500_read,

  /** Perform a Clear-Channel Assessment (CCA) to find out if there is
      a packet in the air or not. */
  cc2500_channel_clear,

  /** Check if the radio driver is currently receiving a packet */
  cc2500_receiving_packet,

  /** Check if the radio driver has just received a packet */
  cc2500_pending_packet,

  /** Turn the radio on. */
  cc2500_on,

  /** Turn the radio off. */
  cc2500_off,

  cc2500_close,
};

/* configurations for the radio driver */
#if USE_WOR
const struct radio_config cc2500_config =
{
	/** Set WOR event0 period. */
	cc2500_set_WORevent0,

	/** Set RX timeout for WOR. */
	cc2500_set_RXtimeout,

	/** Enable RC Oscilator Calibration (Idle to RX). */
	cc2500_enable_RCOSC_CAL,

	/** Disable RC Oscilator Calibration (Timed calibration only). */
	cc2500_disable_RCOSC_CAL,
};
#endif /* USE_WOR */


/*---------------------------------------------------------------------------*/
/* turn on radio */
static void
on(void)
{
#if USE_WOR
  cc2500_strobe(CC2500_SIDLE);
  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
#endif /* USE_WOR */
  cc2500_strobe(CC2500_SRX);
  BUSYWAIT_UNTIL((CC2500_STATUS() == CC2500_STATE_RX), SYSTEM_CONF_SECOND / 100);
  osMutexWait(cc2500_mutex, osWaitForever);
  cc2500_is_on = 1;
  osMutexRelease(cc2500_mutex);
}
/*---------------------------------------------------------------------------*/
/* turn off radio */
static void
off(void)
{
  /* Wait for transmission to end and not receiving §before turning radio off. */
  BUSYWAIT_UNTIL((CC2500_STATUS() != CC2500_STATE_TX), SYSTEM_CONF_SECOND / 100);

  /* might not have finished transmitting here if something is wrong, so we
   * command it into IDLE anyway. */
	#if USE_WOR
      cc2500_strobe(CC2500_SIDLE);
      BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE,  SYSTEM_CONF_SECOND / 100);
      cc2500_strobe(CC2500_SWOR);
	#else
      cc2500_strobe(CC2500_SIDLE);				//Quizas mejor poner SWOR aqui
	#endif

  /* might not have finished transmitting here if something is wrong, so we
   * command it into IDLE anyway. */
  osMutexWait(cc2500_mutex, osWaitForever);
  cc2500_is_on = 0;
  osMutexRelease(cc2500_mutex);
}
/*---------------------------------------------------------------------------*/
void
cc2500_reset(void)
{
  uint8_t i;

  /* reset radio core; after a SRES it takes a little while for the core to get
    ready, keep reading out until chip ready bit is de-asserted */
  cc2500_strobe(CC2500_SRES);
  while(cc2500_strobe(CC2500_SNOP) & 0x80) { }

  /* do basic default setup; see cc2500-config.h */
  for(i = 0; i < CC2500_DEF_CONF_LEN; i += 2) {
    cc2500_write_single(cc2500_default_config[i], cc2500_default_config[i+1]);
  }

  /* calibrate the freq oscillator */
  cc2500_strobe(CC2500_SIDLE);
  cc2500_set_channel(RF_CHANNEL);
  cc2500_strobe(CC2500_SIDLE);

  /* set channel, txp, addr etc */
  CC2500_SET_TXPOWER(CC2500_DEFAULT_TXPOWER);
  cc2500_strobe(CC2500_SCAL);
  BUSYWAIT_UNTIL((CC2500_STATUS() != CC2500_STATE_CAL), SYSTEM_CONF_SECOND / 100);

#if USE_HW_ADDRESS_FILTER
  /* write node address and address filter on (ADDR, broadcast 0x00) */
  cc2500_write_single(CC2500_ADDR, linkaddr_node_addr.u8[0]);
  cc2500_write_single(CC2500_PKTCTRL1, 0x0E);	//HW_ADDRESS_FILTER & CRC Autoflush 0x0E
  //cc2500_write_single(CC2500_PKTCTRL1, 0x0D);	//HW_ADDRESS_FILTER (no BC) & CRC Autoflush
#endif /* USE_HW_ADDRESS_FILTER */

#if USE_HW_ADDRESS_FILTER_LOW
  /* write node address and address filter on (ADDR, broadcast 0x00) */
  cc2500_write_single(CC2500_ADDR, NODEID_INFOMEM_LOCATION[2]);
  cc2500_write_single(CC2500_PKTCTRL1, 0x0E);	//HW_ADDRESS_FILTER & CRC Autoflush
  //cc2500_write_single(CC2500_PKTCTRL1, 0x0D);	//HW_ADDRESS_FILTER (no BC) & CRC Autoflush
#endif /* USE_HW_ADDRESS_FILTER_LOW */


#if USE_WOR
  /* set RXOFF_MODE to IDLE*/
//  PRINTF("CC2500_reset: setting WOR configuration registers\r\n");
  cc2500_write_single(CC2500_MCSM1, 0x3F);	//RXOFF stays in RX and TXOFF to RX
  /* write WOR RX duty cycle*/
  cc2500_write_single(CC2500_MCSM2, 0x02);	//RX timeout, duty cycle= 3.125% 0x02
  cc2500_write_single(CC2500_WOREVT1, 0x87);//write evt1, 1s
  cc2500_write_single(CC2500_WOREVT0, 0x6B);//write evt0
  /* write WOR EVENT1 period, WOR_RES resolution and enable RC oscillator and calibration */
  cc2500_write_single(CC2500_WORCTRL, 0x08);
#endif /* USE_WOR */


  /* start in rx mode or wor mode */
#if USE_WOR
  cc2500_strobe(CC2500_SWORRST);
  cc2500_strobe(CC2500_SWOR);
#else
  cc2500_strobe(CC2500_SRX);
#endif /* USE_WOR */
  osMutexWait(cc2500_mutex, osWaitForever);
  cc2500_is_on = 1;
  cc2500_should_off = 0;
  osMutexRelease(cc2500_mutex);
}

/*---------------------------------------------------------------------------*/
int
cc2500_init(void)
{
	/* Initalize ports and SPI. */
	/* HARDWARE DEPENDENT */
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = CC2500_SPI_CS_PIN;					//CS PIN
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(CC2500_SPI_CS_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = CC2500_GDO2_IRQ_PIN;					//GDO2 PIN
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CC2500_GDO2_IRQ_PORT, &GPIO_InitStruct);

	CC2500_SPI_DISABLE();
	MX_SPI2_Init();

	cc2500_mutex = osMutexCreate(osMutex(cc2500_mutex));

	/* do a default setup of the radio */
	cc2500_reset();

	/* start process that will handle interrupts */
	//  process_start(&cc2500_process, NULL);

	osThreadDef(cc2500Task, cc2500Task_function, osPriorityAboveNormal, 0, 320);
	cc2500TaskId = osThreadCreate(osThread(cc2500Task), NULL);

	/*Allow Interrupts*/
	/* HARDWARE DEPENDENT */
	__HAL_GPIO_EXTI_CLEAR_IT(CC2500_GDO2_IRQ_PIN);
	HAL_NVIC_SetPriority(CC2500_EXT_INTERRUPT, 9, 0);			//ENABLE INTERRUPT
	HAL_NVIC_EnableIRQ(CC2500_EXT_INTERRUPT);
	/* HARDWARE DEPENDENT */
	return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_transmit(unsigned short payload_len)
{
  /* set txp according to the packetbuf attribute */
  if(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER) > 0) {
    CC2500_SET_TXPOWER(packetbuf_attr(PACKETBUF_ATTR_RADIO_TXPOWER));
  } else {
	CC2500_SET_TXPOWER(CC2500_DEFAULT_TXPOWER);
  }


//	CC2500_SET_TXPOWER(cc2500_tx_power);


#if WITH_SEND_CCA
  if(CC2500_STATUS() != CC2500_STATE_RX) {
    cc2500_strobe(CC2500_SRX);
    BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_RX, SYSTEM_CONF_SECOND / 100);
  }
  // XXX wait a little while (?) then do CCA
  BUSYWAIT_UNTIL(0, CCA_BEFORE_TX_TIME);
  if(!CCA) {
    return RADIO_TX_COLLISION;
  }
#endif /* WITH_SEND_CCA */

  /* transmit: strobe Tx, then wait until transmitting, then wait till done */
  cc2500_strobe(CC2500_STX);
  BUSYWAIT_UNTIL((CC2500_STATUS() == CC2500_STATE_TX), SYSTEM_CONF_SECOND / 20);
  BUSYWAIT_UNTIL((CC2500_STATUS() != CC2500_STATE_TX), SYSTEM_CONF_SECOND / 20);

#if !WOR
  cc2500_strobe(CC2500_SIDLE);
  cc2500_strobe(CC2500_SFRX);
  cc2500_strobe(CC2500_SRX);
#endif
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_prepare(const void *payload, unsigned short payload_len)
{
  /* Write packet to TX FIFO after flushing it. First byte is total length
    (hdr+data), not including the length byte. */

  cc2500_strobe(CC2500_SIDLE);
  cc2500_strobe(CC2500_SFTX);

//  cc2500_write_burst(CC2500_TXFIFO, (uint8_t *)&payload_len, 1);

#if USE_HW_ADDRESS_FILTER || USE_HW_ADDRESS_FILTER_LOW
  /* If using HW address filter, address byte length needs to be added to total packet length */
  unsigned short packet_len = payload_len + 1;
  cc2500_write_burst(CC2500_TXFIFO, (uint8_t *)&packet_len, 1);
#else
  cc2500_write_burst(CC2500_TXFIFO, (uint8_t *)&payload_len, 1);
#endif /* USE_HW_ADDRESS_FILTER */

#if USE_HW_ADDRESS_FILTER
  /* write destination address high byte to FIFO to use HW address filtering */
  {
	linkaddr_t dest;
    dest = *packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
    cc2500_write_burst(CC2500_TXFIFO, &(dest.u8[0]), 1);
//    PRINTF("cc2500: Sending to HW address: %d\r\n", dest.u8[0]);
  }
#endif /* USE_HW_ADDRESS_FILTER */

#if USE_HW_ADDRESS_FILTER_LOW
  /* write destination address high byte to FIFO to use HW address filtering */
  {
    cc2500_write_burst(CC2500_TXFIFO, &(hw_dest), 1);
    if (test_count == 0)
    {
    	PRINTF("Sending to HW address: %d\r\n", hw_dest);
    	test_count++;
    }
  }
#endif /* USE_HW_ADDRESS_FILTER_LOW */

  cc2500_write_burst(CC2500_TXFIFO, (uint8_t*) payload, payload_len);

  return 0;
}
/*---------------------------------------------------------------------------*/
int
cc2500_send(const void *payload, unsigned short payload_len)
{
  cc2500_prepare(payload, payload_len);

//  char *print = (char*) payload;
//  uint8_t i = 0;
//  uint8_t received[64];
//  for (i =0; i<payload_len; i++){
//	  received[i] = print[i];
//  }
//  i = 0;

  return cc2500_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
int
cc2500_off(void)
{
  /* Don't do anything if we are already turned off. */
  osMutexWait(cc2500_mutex, osWaitForever);
  if(!cc2500_is_on) {
	  osMutexRelease(cc2500_mutex);
    return 1;
  }

  /* if we are transmitting or currently recieving, don't turn off */
  if(CC2500_STATUS() == CC2500_STATE_TX) {
    osMutexWait(cc2500_mutex, osWaitForever);
    cc2500_should_off = 1;
    osMutexRelease(cc2500_mutex);
    return 0;
  }

  off();
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2500_on(void)
{
  osMutexWait(cc2500_mutex, osWaitForever);
  cc2500_should_off = 0;
  osMutexRelease(cc2500_mutex);
  on();
  return 1;
}
/*---------------------------------------------------------------------------*/
int cc2500_close(void){

	/* Initalize ports and SPI. */

		HAL_NVIC_DisableIRQ(CC2500_EXT_INTERRUPT);//Disable INTERRUPT

		CC2500_SPI_DISABLE();

		osMutexDelete(cc2500_mutex);

		/* do a default setup of the radio */
		cc2500_strobe(CC2500_SFRX);
		cc2500_strobe(CC2500_SFTX);
		cc2500_strobe(CC2500_SIDLE);
		cc2500_strobe(CC2500_SRES);
		while(cc2500_strobe(CC2500_SNOP) & 0x80) { }
		cc2500_strobe(CC2500_SPWD);

		CC2500_SPI_DISABLE();

		osThreadTerminate(cc2500TaskId);

		return 0;
}
/*---------------------------------------------------------------------------*/
void
cc2500_set_channel(uint8_t c)
{
  /* Wait for any ev transmission to end and any receiving to end. */
  BUSYWAIT_UNTIL((CC2500_STATUS() != CC2500_STATE_TX), SYSTEM_CONF_SECOND / 100);

  /* need to be in Idle or off, stable. */
  if(CC2500_STATUS() != CC2500_STATE_IDLE) {
    cc2500_strobe(CC2500_SIDLE);
  }

  /* write channel setting */
  cc2500_write_single(CC2500_CHANNR, c);

  /* calibrate oscillator for this new freq */
  cc2500_strobe(CC2500_SCAL);
  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);

  /* turn on radio if it was on */
  if(cc2500_is_on) {
    on();
  } else {
    off();
  }
  return;
}
/*---------------------------------------------------------------------------*/
/* this is called from the interrupt service routine; polls the radio process
  which in turn reads the packet from the radio when it runs. The reason for
  this is to avoid blocking the radio from the ISR. */
volatile uint8_t cc2500_pending_rxfifo = 0;  // XXX
int
cc2500_interrupt(void)
{
  cc2500_pending_rxfifo++;
  if (cc2500TaskId != NULL){
	  osThreadResume (cc2500TaskId);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
void cc2500Task_function(void const * argument){


	cc2500TaskId = osThreadGetId();
/*  timer_set(&radio_check_timer, CLOCK_SECOND);*/
  while(1) {
	  osThreadSuspend(cc2500TaskId);

#if USE_WOR
    if (RSOSC_disabled == 1)
    	{
    	cc2500_RCOSC_CALIBRATE();
    	}
#endif /* USE_WOR */

    /* We end up here after a radio GDO port ISR -> interrupt handler -> poll process */
    static uint8_t len = 0;
    //usb_printf("CC2500 polled\r\n");
    // cc2500_pending_rxfifo--;    /* mli: TODO: pendingfix */

    cc2500_read_burst(CC2500_RXBYTES, &len, 1);

    /* overflow in RxFIFO, drop all */
    if(len & 0x80) {
      FLUSH_FIFOS();
#if USE_WOR
      if (!cc2500_is_on){
            cc2500_strobe(CC2500_SIDLE);
            BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
            cc2500_strobe(CC2500_SWOR);
          } else {
          	cc2500_strobe(CC2500_SRX);
          }
	#else
      cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */

    } else if(len > 0) {
      /* prepare packetbuffer */
      packetbuf_clear();

      /* read length of packet (first FIFO byte) then pass to higher layers */
      len = cc2500_read(packetbuf_dataptr(), PACKETBUF_SIZE);

      if(len > 0) {
        packetbuf_set_datalen(len);

        NETSTACK_RDC.input();


        /* re-poll the radio process so it can check for any packet received while
          we were handling this one (it will check the rxfifo for data). */
/*          cc2500_interrupt();*/
      } else {
        /* no received data or bad data that was dropped. Do nothing. */
      }
    }

    /* LPM4 for MSP430, change for other mcu architectures*/
    #if USE_WOR
    if (!cc2500_is_on){
    		cc2500_strobe(CC2500_SIDLE);
    	    BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
    	    cc2500_strobe(CC2500_SWOR);
//    		LPM4;
            }
	#else
    cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */
  }
}
/*---------------------------------------------------------------------------*/
static int
cc2500_read(void *buf, unsigned short bufsize)
{
  uint8_t footer[2];
  uint8_t len;

  len = cc2500_read_single(CC2500_RXBYTES);


    if(cc2500_pending_rxfifo == 0) {
      /* we don't have anything in the FIFO (ie no interrupt has fired) */
      if(len > 0) {
        FLUSH_FIFOS();
  	#if USE_WOR
        if (!cc2500_is_on){
        cc2500_strobe(CC2500_SIDLE);
        BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, RTIMER_SECOND / 100);
        cc2500_strobe(CC2500_SWOR);
      } else {
      	cc2500_strobe(CC2500_SRX);
      }
  	#else
        cc2500_strobe(CC2500_SRX);
  	#endif /* USE_WOR */
      }
      return 0;
    }

  cc2500_pending_rxfifo = 0;

  /* check for FIFO overflow */
  if(len & 0x80) {
	    /* overflow in RxFIFO, drop all */
	FLUSH_FIFOS();
//	PRINTF("Overflow;F\r\n");
	#if USE_WOR
	if (!cc2500_is_on){
	  cc2500_strobe(CC2500_SIDLE);
	  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
	  cc2500_strobe(CC2500_SWOR);
	} else {
		cc2500_strobe(CC2500_SRX);
	}
	#else
	  cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */
	return 0;
  } else if(len == 0) {
    /* nothing in buffer */
    return 0;
  }

  /* first byte in FIFO is length of the packet with no appended footer */
  cc2500_read_burst(CC2500_RXFIFO, &len, 1);
//  usb_printf("%u B\r\n", len);


  /* Check size; too small (ie no real "data") -> drop it */
  if(len == 0) {
	  FLUSH_FIFOS();
//		  PRINTF("No data;F\r\n");
		#if USE_WOR
			if (!cc2500_is_on){
				cc2500_strobe(CC2500_SIDLE);
				BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
				cc2500_strobe(CC2500_SWOR);
			  } else {
				cc2500_strobe(CC2500_SRX);
			  }
		#else
			cc2500_strobe(CC2500_SRX);
		#endif /* USE_WOR */
	return 0;
  }

  /* corrupt size -> drop it */
  if(len > CC2500_MAX_PACKET_LEN) {
	FLUSH_FIFOS();
//	PRINTF("Bad len, greater than 64;F\r\n");
	#if USE_WOR
		if (!cc2500_is_on){
		  cc2500_strobe(CC2500_SIDLE);
		  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
		  cc2500_strobe(CC2500_SWOR);
		} else {
			cc2500_strobe(CC2500_SRX);
		}
	#else
	  cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */
	return 0;
  }

#if USE_HW_ADDRESS_FILTER || USE_HW_ADDRESS_FILTER_LOW
  len--;	//BLS: HW address is included in the length
#endif /* USE_HW_ADDRESS_FILTER */

  /* Check size; too big for buffer -> drop it */
  if(len > bufsize) {
	  FLUSH_FIFOS();
//	  PRINTF("len: %u\r\n", len);
//	  PRINTF("Too big(%u);F\r\n", bufsize);
	#if USE_WOR
		if (!cc2500_is_on){
			cc2500_strobe(CC2500_SIDLE);
			BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
			cc2500_strobe(CC2500_SWOR);
		  } else {
			cc2500_strobe(CC2500_SRX);
		  }
	#else
		cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */
	  return 0;
  }


#if USE_HW_ADDRESS_FILTER
  /* get destination address high byte to determine if to be ACKed (later) */
  uint8_t dest;

  cc2500_read_burst(CC2500_RXFIFO, &dest, 1);
//  PRINTF("cc2500_read: Destination: %u\r\n", dest);
  if((dest != linkaddr_node_addr.u8[0]) && (dest != 0xFF) && (dest != 0x00))
  {
	  FLUSH_FIFOS();
//	  PRINTF("cc2500: Wrong address;F\r\n");
	#if USE_WOR
	  	  if (!cc2500_is_on){
	        cc2500_strobe(CC2500_SIDLE);
	        BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
	        cc2500_strobe(CC2500_SWOR);
	      } else {
	      	cc2500_strobe(CC2500_SRX);
	      }
	#else
      cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */
	  return 0;
  }
#endif /* USE_HW_ADDRESS_FILTER */

#if USE_HW_ADDRESS_FILTER_LOW
  /* get destination address high byte to determine if to be ACKed (later) */
  uint8_t dest;
  cc2500_read_burst(CC2500_RXFIFO, &dest, 1);
  PRINTF("cc2500_read: Destination: %u\r\n", dest);
  if((dest != NODEID_INFOMEM_LOCATION[2]) && (dest != 0xFF) && (dest != 0x00))
  {
	  FLUSH_FIFOS();
	  PRINTF("cc2500: Wrong address;F\r\n");
	#if USE_WOR
	  	  if (!cc2500_is_on){
	        cc2500_strobe(CC2500_SIDLE);
	        BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
	        cc2500_strobe(CC2500_SWOR);
	      } else {
	      	cc2500_strobe(CC2500_SRX);
	      }
	#else
      cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */
	  return 0;
  }
#endif /* USE_HW_ADDRESS_FILTER */

  /* The complete packet needs to be in the FIFO before being read (without this, large packets
   * are not correctly read) BLS*/
  BUSYWAIT_UNTIL((cc2500_read_single(CC2500_RXBYTES) == (len + 2)), SYSTEM_CONF_SECOND/100);

  /* read the packet data from RxFIFO, put in packetbuf */

  cc2500_read_burst(CC2500_RXFIFO, buf, len);

//////////////BQTEST////////////////////////////
//  char *print = (char*) buf;
//  uint8_t i = 0;
//  uint8_t received[64];
//  for (i =0; i<len; i++){
//	  received[i] = print[i];
//  }
//  i = 0;
//if((print[0]=='T') & (print[1]=='E') & (print[2]=='S') &(print[3]=='T')){
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4| GPIO_PIN_5);
//	clock_wait(150);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4| GPIO_PIN_5);
//}
//////////////BQTEST////////////////////////////
//  CC2500_READ_FIFO_BUF(buf, len);
//  char *print = (char*) buf;
//usb_printf("%s\r\n", print);

/*
for(i=0; i<16; i++){
test = cc2500_read_single(CC2500_RXFIFO);
usb_printf("Chars read: 0x%x\r\n", test);
}
*/



  /* read automatically appended data (RSSI, LQI, CRC ok) */
  if(FOOTER_LEN > 0) {
	cc2500_read_burst(CC2500_RXFIFO, footer, FOOTER_LEN);
//    CC2500_READ_FIFO_BUF(footer, FOOTER_LEN);
    if(footer[1] & FOOTER1_CRC_OK) {
      /* set attributes: RSSI and LQI so they can be read out from packetbuf */
      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, footer[0]);
      packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, footer[1] & FOOTER1_LQI);
    } else {
	 /* CRC fail -> drop packet */
		  FLUSH_FIFOS();
//		  PRINTF("CRC fail;F\r\n");
		#if USE_WOR
			  if (!cc2500_is_on){
				cc2500_strobe(CC2500_SIDLE);
				BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
				cc2500_strobe(CC2500_SWOR);
			  } else {
				cc2500_strobe(CC2500_SRX);
			  }
		#else
		  cc2500_strobe(CC2500_SRX);
		#endif /* USE_WOR */
		  return 0;
    }
  }

  /* As the radio defaults to idle after receiving, strobe RX again */
	#if USE_WOR
	  if (!cc2500_is_on){
		cc2500_strobe(CC2500_SIDLE);
		BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
		cc2500_strobe(CC2500_SWOR);
	  } else {
		cc2500_strobe(CC2500_SRX);
	  }
	#else
	  cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */

  /* check for FIFO overflow or if anything else is left in FIFO */
  if(CC2500_STATUS() == CC2500_STATE_RXFIFO_OVERFLOW) {
	FLUSH_FIFOS();
//	PRINTF("cc2500 read: Overflow\r\n");
	#if USE_WOR
		if (!cc2500_is_on){
		  cc2500_strobe(CC2500_SIDLE);
		  BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
		  cc2500_strobe(CC2500_SWOR);
		} else {
			cc2500_strobe(CC2500_SRX);
		}
	#else
	  cc2500_strobe(CC2500_SRX);
	#endif /* USE_WOR */
  } else {
    uint8_t len = cc2500_read_single(CC2500_RXBYTES);
    if(len > 0) {
       //XXX another packet in buffer, handle it
      cc2500_pending_rxfifo++;
      if (cc2500TaskId != NULL){
    	  osThreadResume (cc2500TaskId);
      }
    }
  }

#if USE_HW_ADDRESS_FILTER
  /*
   * check if this is a unicast, if so we ACK it (not broadcast or to us are
   * already filtered out by radio HW).
   * Broadcasts are sent to address XX
   *
   *
   * move earlier to save sender energy? if not interferes with reading rxfifo
   * --no, we should do the previous checks to see that it was received properly
   */

  if(dest == linkaddr_node_addr.u8[0]) {
#if 0
	  //PRINTF("buf [0] = %02X\r\n", buf[0]);
	  //PRINTF("buf [1] = %02X\r\n", buf[1]);
	  PRINTF("cc2500_read: buffer = ");
	        uint8_t *content;
	        int cont_len;

	        content = buf;
	        cont_len = bufsize;

	        int i;

	        for(i = 0; i < cont_len; ++i) {
	      	  PRINTF("%c", content[i]);
	            }
	        PRINTF("\r\n");
#endif /* DEBUG */
    /* BLS: HW ACK not yet implemented
    uint8_t ab[ACK_LEN];  // ACK-buffer

    ab[0] = rimeaddr_node_addr.u8[0];
    ab[1] = rimeaddr_node_addr.u8[1];
    ab[2] = tx_serial;    // XXX what's here

    // XXX send the ACK
    PRINTF("CC2500 Sent ACK!\r\n");
    */
  }

#endif /* USE_HW_ADDRESS_FILTER */

#if USE_HW_ADDRESS_FILTER_LOW
  if(dest == hw_dest) {
#if 0
	  PRINTF("cc2500_read: buffer = ");
	        uint8_t *content;
	        int cont_len;

	        content = buf;
	        cont_len = bufsize;

	        int i;

	        for(i = 0; i < cont_len; ++i) {
	      	  PRINTF("%c", content[i]);
	            }
	        PRINTF("\r\n");
#endif /* DEBUG */
    /* BLS: HW ACK not yet implemented
    uint8_t ab[ACK_LEN];  // ACK-buffer

    ab[0] = rimeaddr_node_addr.u8[0];
    ab[1] = rimeaddr_node_addr.u8[1];
    ab[2] = tx_serial;    // XXX what's here

    // XXX send the ACK
    PRINTF("CC2500 Sent ACK!\r\n");
    */
}

#endif /* USE_HW_ADDRESS_FILTER_LOW */


  #if USE_WOR
	if (!cc2500_is_on){
		cc2500_strobe(CC2500_SIDLE);
		BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
		cc2500_strobe(CC2500_SWOR);
		} else {
			cc2500_strobe(CC2500_SRX);
		}
  #endif /* USE_WOR */

  return len;
}
/*---------------------------------------------------------------------------*/
#if 0
void
cc2500_set_txpower(uint8_t power)
{
  cc2500_write_single(CC2500_PATABLE, power);
}
#endif
/*---------------------------------------------------------------------------*/
/* read and return current RSSI */
int
cc2500_rssi(void)
{
  int rssi;
  int radio_was_off = 0;

/*  if(locked) {*/
/*    return 0;*/
/*  }*/

  if(!cc2500_is_on) {
    radio_was_off = 1;
    cc2500_on();    // XXX on()? no, the cc2500_on will wait for stable on (?)
  }

  rssi = (int)((signed char)cc2500_read_single(CC2500_RSSI));

  if(radio_was_off) {
    cc2500_off();
  }
  return rssi;
}
/*---------------------------------------------------------------------------*/
int
cc2500_channel_clear(void)
{
  volatile uint8_t cca = 0;
  uint8_t radio_was_off = 0;

  /* If the radio is locked by an underlying thread (because we are
     being invoked through an interrupt), we pretend that the coast is
     clear (i.e., no packet is currently being transmitted by a
     neighbor). */
/*  if(locked) {*/
/*    return 1;*/
/*  }*/

  if(!cc2500_is_on) {
    radio_was_off = 1;
    cc2500_on();
  }

  /* read CCA */
  cca = cc2500_read_single(CC2500_PKTSTATUS);

  if(radio_was_off) {
    cc2500_off();
  }

  /* XXX : bug, was always returning false strangely enough, so manually override it here. */
  /* should read out eg 0x30 where CCA is 1 << 4, and saw this on the logic analyzer too.... */
  /* BLS: uncommented, if timer is set to let the RSSI reading settle, the cca reading is reliable */

  if(cca & PKTSTATUS_CCA) {
//    	PRINTF("cclear\r\n");
    	return 1;
  } else {
//	  PRINTF("cnclear\r\n");
	  return 0;
  }

}
/*---------------------------------------------------------------------------*/
int
cc2500_receiving_packet(void)
{
  /* if GDO is high, it means we are either receiving or sending a packet */
  if(/*(CC2500_GDO_PORT(IN) & CC2500_GDO_PIN) &&*/ (CC2500_STATUS() != CC2500_STATE_TX)) {
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
cc2500_pending_packet(void)
{
  return (int) cc2500_read_single(CC2500_RXBYTES);
  // return cc2500_pending_rxfifo;
}
/*--------------------------------------------------------------------------*/
uint8_t
cc2500_strobe(uint8_t strobe)
{
  uint8_t s;
  /* sth (the button?) sets the MISO pin so lets reset all pins we need */
//  P3SEL  |= SPI_MISO | SPI_MOSI | SPI_CLK;
  //SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  CC2500_SPI_ENABLE();
  osMutexWait(spi2_mutex, osWaitForever);
  HAL_SPI_TransmitReceive(&hspi2, &strobe, &s, 1, 10);
  osMutexRelease(spi2_mutex);
//  SPI_WRITE(strobe);
//  s = SPI_RXBUF;
  CC2500_SPI_DISABLE();
//  usb_printf("STATUS: 0x%02x\r\n", s);
  return s;
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_read_single(uint8_t adr)
{
  uint8_t data[2];
  uint8_t send[2];
  send[0] = adr | CC2500_READ;


  if(adr >= CC2500_PARTNUM) {
    /* these regs can only be read in burst mode and one at a time */
    cc2500_read_burst(adr, data, 1);
    return data[0];
  } else {
    CC2500_SPI_ENABLE();

    osMutexWait(spi2_mutex, osWaitForever);
    HAL_SPI_TransmitReceive(&hspi2, send, data, 2, 10);		//Transmit dir and dummy and read
    osMutexRelease(spi2_mutex);

    CC2500_SPI_DISABLE();
    return data[1];
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_read_burst(uint8_t adr, uint8_t *dest, uint8_t len)
{
  uint8_t send[len+1];
  send[0] = adr | CC2500_BURSTREAD;
  uint8_t rcv[len+1];
  uint16_t i = 0;

  CC2500_SPI_ENABLE();

  osMutexWait(spi2_mutex, osWaitForever);
  HAL_SPI_TransmitReceive(&hspi2, send, rcv, (uint16_t) len+1, 10);
  osMutexRelease(spi2_mutex);

  CC2500_SPI_DISABLE();

  for(i=0; i<len; i++){
	  dest[i] = rcv[i+1];
  }

  return cc2500_strobe(CC2500_SNOP);
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_write_single(uint8_t adr, uint8_t data)
{
  uint8_t s[2];
  uint8_t send[2];
  send[0] = adr | CC2500_WRITE;
  send[1] = data;

  CC2500_SPI_ENABLE();

  osMutexWait(spi2_mutex, osWaitForever);
  HAL_SPI_TransmitReceive(&hspi2, send, s, 2, 10);
  osMutexRelease(spi2_mutex);

  CC2500_SPI_DISABLE();
  return s[0];
}
/*---------------------------------------------------------------------------*/
uint8_t
cc2500_write_burst(uint8_t adr, uint8_t *src, uint8_t len)
{
	uint8_t s[65];				//MAX FIFO SIZE 64
	uint8_t send[65];
	send[0] = adr | CC2500_BURSTWRITE;
	uint16_t i = 0;

	for(i=0; i<len; i++){
		send[i+1] = src[i];
	}

  CC2500_SPI_ENABLE();
  osMutexWait(spi2_mutex, osWaitForever);
  HAL_SPI_TransmitReceive(&hspi2, send, s, (uint16_t) len+1, 10);
  osMutexRelease(spi2_mutex);

  CC2500_SPI_DISABLE();
  return s[0];
}
/*---------------------------------------------------------------------------*/
/* Checks whether the radio is still in RX mode, so we can reset it otherwise. 
Expect this to be removed later. */
int
cc2500_radio_ok(void)
{
  if(CC2500_STATUS() == CC2500_STATE_RX) {
    return 0;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
int
cc2500_calibrate(void)
{
  cc2500_strobe(CC2500_SIDLE);
  cc2500_strobe(CC2500_SCAL);
  return 1;
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
#if USE_HW_ADDRESS_FILTER_LOW
void	cc2500_set_hw_dest(uint8_t dest)
{
	hw_dest = dest;
	PRINTF("HW Address dest: %02X\r\n", hw_dest);
}
#endif /* USE_HW_ADDRESS_FILTER */
/*---------------------------------------------------------------------------*/
#if USE_WOR
/* Set WOR Event0 period. tevent0 = 750/26MHz * event0 = 28.846x10^-6 * event0
 * min time = 11.08ms + RX timeout + tevent1 (0.111-0.115ms)
 * max time = 1.89s*/
int
cc2500_set_WORevent0(uint16_t event0)
{
	uint8_t evt1 = (uint8_t)(event0>>8);
	uint8_t evt0 = (uint8_t)event0;
	cc2500_strobe(CC2500_SIDLE);
	BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
	cc2500_write_single(CC2500_WOREVT1, evt1);
	cc2500_write_single(CC2500_WOREVT0, evt0);
	cc2500_strobe(CC2500_SWOR);
//	PRINTF("cc2500_config: set WORevent0 to 0x%02X%02X\r\n", evt1, evt0);
	return 1;
}
/*---------------------------------------------------------------------------*/
/* Set RX timeout for WOR*/
int
cc2500_set_RXtimeout(uint8_t rx_timeout)
{
	cc2500_strobe(CC2500_SIDLE);
	BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
	uint8_t mcsm2_status = cc2500_read_single(CC2500_MCSM2);
	uint8_t rxt = ((mcsm2_status & 0xF8) | (rx_timeout & 07));
	cc2500_write_single(CC2500_MCSM2, rxt);
	if (!cc2500_is_on) {
		cc2500_strobe(CC2500_SWOR);
	} else {
		cc2500_strobe(CC2500_SRX);
	}
//	PRINTF("cc2500_config: set RX timeout (MCSM2) to 0x%02X\r\n", rxt);
	return 1;
}
/*---------------------------------------------------------------------------*/
/** Enable RC Oscilator Calibration (Idle to RX). */
int
cc2500_enable_RCOSC_CAL(void)
{
	cc2500_strobe(CC2500_SIDLE);
	BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
	uint8_t worctrl_status = cc2500_read_single(CC2500_WORCTRL);
	uint8_t worctrl_new = worctrl_status | 0x08;
	cc2500_write_single(CC2500_WORCTRL, worctrl_new);
	cc2500_strobe(CC2500_SWOR);
//	PRINTF("cc2500_config: enabled RC OSC Calibration, WOR_CTRL = %02x\r\n", worctrl_new);
	RSOSC_disabled = 0;
	return 1;
}
/*---------------------------------------------------------------------------*/
/** Disable RC Oscilator Calibration (Timed calibration only). */
int
cc2500_disable_RCOSC_CAL(void)
{
	uint8_t calib1;
	uint8_t calib0;
	cc2500_strobe(CC2500_SIDLE);
	BUSYWAIT_UNTIL(CC2500_STATUS() == CC2500_STATE_IDLE, SYSTEM_CONF_SECOND / 100);
//	PRINTF("CC2500_disable RCOSC: Idle\r\n");
	BUSYWAIT_UNTIL(0, SYSTEM_CONF_SECOND / 150);	//BLS: RTIMER WAS NOT WORKING.. Need to re-test
	// Wait for RCOSC calibration
//	PRINTF("CC2500_disable RCOSC: Finished calibrating RCOSC\r\n");
	uint8_t worctrl_status = cc2500_read_single(CC2500_WORCTRL);
	uint8_t worctrl_new = worctrl_status & 0xF7;
	cc2500_write_single(CC2500_WORCTRL, worctrl_new); // EVENT1 = 3
	// RC_CAL = 0
	// WOR_RES = 0
	calib1 = cc2500_read_single(CC2500_RCCTRL1_STATUS);
	calib0 = cc2500_read_single(CC2500_RCCTRL0_STATUS);
	cc2500_write_single(CC2500_RCCTRL1, calib1);
	cc2500_write_single(CC2500_RCCTRL0, calib0);
	cc2500_strobe(CC2500_SWORRST);
	cc2500_strobe(CC2500_SWOR);
	RSOSC_disabled = 1;
//	PRINTF("CC2500_config: disabled RC OSC Calibration, WOR_CTRL = %02x\r\n", worctrl_new);
	return 1;
}
/*---------------------------------------------------------------------------*/
void
cc2500_RCOSC_CALIBRATE(void)
{
	uint8_t calib1;
	uint8_t calib0;

	uint8_t worctrl_status = cc2500_read_single(CC2500_WORCTRL);
	uint8_t worctrl_new = worctrl_status | 0x08;
	cc2500_write_single(CC2500_WORCTRL, worctrl_new);
//	PRINTF("CC2500_config: RC OSC Calibration, WOR_CTRL = %02x\r\n", worctrl_new);
	cc2500_strobe(CC2500_SWORRST);
	BUSYWAIT_UNTIL(0, SYSTEM_CONF_SECOND / 150);	// Wait for RCOSC calibration
	worctrl_new = worctrl_status & 0xF7;
	cc2500_write_single(CC2500_WORCTRL, worctrl_new); 	// RC_CAL = 0
//	PRINTF("cc2500_config: RC OSC Calibration, WOR_CTRL = %02x\r\n", worctrl_new);
	calib1 = cc2500_read_single(CC2500_RCCTRL1_STATUS);
	calib0 = cc2500_read_single(CC2500_RCCTRL0_STATUS);
	cc2500_write_single(CC2500_RCCTRL1, calib1);
	cc2500_write_single(CC2500_RCCTRL0, calib0);
	cc2500_strobe(CC2500_SWORRST);
	cc2500_strobe(CC2500_SWOR);
//	PRINTF("RSOSC calibrated\r\n");
}
#endif /* USE_WOR */
/*---------------------------------------------------------------------------*/

