/*
 * Copyright (c) 2012, STMicroelectronics.
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
 *
 */
/*---------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "spi.h"
#include "rime.h"
#include "netstack.h"
#include "spirit1.h"
#include "stm32l4xx.h"
#include "spirit1-arch.h"
#include "spirit1_appli_433.h"
#include "spirit1_appli_868.h"
#include "spirit1_appli.h"
#include "spirit1.h"
#include "spirit1-arch.h"
#include "spirit1-arch.h"
#include <stdio.h>
#include "timers.h"
#include "MCU_Interface.h"
#include "leds.h"

/*---------------------------------------------------------------------------*/
/* MGR extern st_lib_spirit_irqs st_lib_x_irq_status; */
volatile SpiritFlagStatus rx_timeout;
/*---------------------------------------------------------------------------*/
#define XXX_ACK_WORKAROUND 1
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/

extern radio_driver_state_t current_radio_driver;

#define RECEIVE_TIMEOUT	1000

void receive_timeout  (void const *arg);

osTimerDef(receive_timer, receive_timeout);  // when the timer expires, the function start_machine is called
osTimerId receive_timer_id;


/*---------------------------------------------------------------------------*/
void spirit1Task_function(void const * argument);

uint8_t interrupt_thread_routine(void);

osThreadId spirit1TaskId = NULL;

/*---------------------------------------------------------------------------*/
//#define CLEAR_TXBUF()           (spirit_txbuf[0] = 0)
#define CLEAR_RXBUF()           //if(rxbuf!=NULL){vPortFree(rxbuf);rxbuf=NULL;}
//#define IS_TXBUF_EMPTY()        (spirit_txbuf[0] == 0)
//#define IS_RXBUF_EMPTY()        (spirit_rxbuf[0] == 0)
//#define IS_RXBUF_FULL()         (spirit_rxbuf[0] != 0)
/*---------------------------------------------------------------------------*/
/* transceiver state. */
#define ON     0
#define OFF    1
/*---------------------------------------------------------------------------*/
static volatile unsigned int spirit_on = OFF;
static volatile uint8_t receiving_packet = 0;
static packetbuf_attr_t last_rssi = 0;  /* MGR */
static packetbuf_attr_t last_lqi = 0;  /* MGR */
/*---------------------------------------------------------------------------*/
static int interrupt_callback_in_progress = 0;
static int interrupt_callback_wants_poll = 0;
/*---------------------------------------------------------------------------*/

/**
* @brief GPIO structure fitting
*/
SGpioInit xGpioIRQ={
  SPIRIT_GPIO_3,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};



/*
 * The buffers which hold incoming data.
 * The +1 because of the first byte,
 * which will contain the length of the packet.
 */
//static uint8_t spirit_rxbuf[MAX_PACKET_LEN + 1];
//static uint8_t spirit_txbuf[MAX_PACKET_LEN + 1 - SPIRIT_MAX_FIFO_LEN];
uint8_t rxbuf[MAX_PACKET_LEN];
uint16_t rcv_packet_len;
uint16_t current_rx_bytes;
uint8_t rcv_new_packet = 1;
uint8_t prev_packet_not_processed = 0;
uint8_t flush_remaining_packet = 0;

uint8_t* spirit_txbuf;
uint16_t remaining_tx_payload_bytes;
uint8_t sent_done = 0;
void SpiritManagementSetFrequencyBase(uint32_t);
/*---------------------------------------------------------------------------*/
static int just_got_an_ack = 0; /* Interrupt callback just detected an ack */
#if NULLRDC_CONF_802154_AUTOACK
#define ACK_LEN 3
static int wants_an_ack = 0; /* The packet sent expects an ack */
/* static int just_got_an_ack = 0; / * Interrupt callback just detected an ack * / */
/* #define ACKPRINTF printf */
#define ACKPRINTF(...)
#endif /* NULLRDC_CONF_802154_AUTOACK */
/*---------------------------------------------------------------------------*/
static int packet_is_prepared = 0;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static int spirit_radio_init(void);
static int spirit_radio_prepare(const void *payload, unsigned short payload_len);
static int spirit_radio_transmit(unsigned short payload_len);
static int spirit_radio_send(const void *data, unsigned short len);
static int spirit_radio_read(void *buf, unsigned short bufsize);
static int spirit_radio_channel_clear(void);
static int spirit_radio_receiving_packet(void);
static int spirit_radio_pending_packet(void);
static int spirit_radio_on(void);
static int spirit_radio_off(void);
static int spirit_radio_close(void);
/*---------------------------------------------------------------------------*/
const struct radio_driver spirit_radio_driver =
{
  spirit_radio_init,
  spirit_radio_prepare,
  spirit_radio_transmit,
  spirit_radio_send,
  spirit_radio_read,
  spirit_radio_channel_clear,
  spirit_radio_receiving_packet,
  spirit_radio_pending_packet,
  spirit_radio_on,
  spirit_radio_off,
  spirit_radio_close,
};
/*---------------------------------------------------------------------------*/


/**
 * @brief Print the spirit1 status
 */
void
spirit1_printstatus(void)
{
  int s = SPIRIT1_STATUS();
  if(s == SPIRIT1_STATE_STANDBY) {
	  usb_printf("spirit1: SPIRIT1_STATE_STANDBY\n");
  } else if(s == SPIRIT1_STATE_READY) {
	  usb_printf("spirit1: SPIRIT1_STATE_READY\n");
  } else if(s == SPIRIT1_STATE_TX) {
	  usb_printf("spirit1: SPIRIT1_STATE_TX\n");
  } else if(s == SPIRIT1_STATE_RX) {
	  usb_printf("spirit1: SPIRIT1_STATE_RX\n");
  } else {
	  usb_printf("spirit1: status: %d\n", s);
  }
}

/**
 * @brief Strobe a command. The rationale for this is to clean up the messy legacy code.
 * @param s
 */
static void
spirit1_strobe(uint8_t s)
{
	RadioSpiCommandStrobes(s);
}

/**
 * @brief Set ready state
 */
void
spirit_set_ready_state(void)
{
  PRINTF("READY IN\n");

  SpiritIrqClearStatus();
  if(current_radio_driver == RADIO_SPIRIT_433){
	IRQ_DISABLE_433();
  }
  else if(current_radio_driver == RADIO_SPIRIT_868){
	IRQ_DISABLE_868();
  }

  if(SPIRIT1_STATUS() == SPIRIT1_STATE_STANDBY) {
    spirit1_strobe(SPIRIT1_STROBE_READY);
  } else if(SPIRIT1_STATUS() == SPIRIT1_STATE_RX) {
    spirit1_strobe(SPIRIT1_STROBE_SABORT);
    SpiritIrqClearStatus();
  }

  if(current_radio_driver == RADIO_SPIRIT_433){
	IRQ_ENABLE_433();
  }
  else if(current_radio_driver == RADIO_SPIRIT_868){
	IRQ_ENABLE_868();
  }

  PRINTF("READY OUT\n");
}

/**
 * @brief Initialize the spirit1 radio
 * @return 0 if success
 */
static int
spirit_radio_init(void)
{
  PRINTF("RADIO INIT IN\n");
  GPIO_InitTypeDef GPIO_InitStruct;

  receive_timer_id = osTimerCreate(osTimer(receive_timer), osTimerOnce, (void *)0);
  /* Configures the SPIRIT1 library */

  /* wake up to READY state */
  /* weirdly enough, this *should* actually *set* the pin, not clear it! The pins is declared as GPIO_pin13 == 0x2000 */
  RadioSpiInit();

  if(current_radio_driver == RADIO_SPIRIT_433){
	  HAL_GPIO_WritePin(SPIRIT_433_GPIO_SDN_PORT, SPIRIT_433_GPIO_SDN_PIN, GPIO_PIN_RESET);
	  osDelay(1);

	  /* Soft reset of core */
	  spirit1_strobe(SPIRIT1_STROBE_SRES);
	  osDelay(5);
	  /* Configures the SPIRIT1 radio part */
	  SRadioInit x_radio_init = {
			  /* XTAL_FREQUENCY, */
			  XTAL_OFFSET_PPM_433,
			  BASE_FREQUENCY_433,
			  CHANNEL_SPACE_433,
			  CHANNEL_NUMBER_433,
			  MODULATION_SELECT_433,
			  DATARATE_433,
			  FREQ_DEVIATION_433,
			  BANDWIDTH_433
	  };

	  SpiritRadioSetXtalFrequency(XTAL_FREQUENCY);
	  SpiritManagementSetFrequencyBase(XTAL_FREQUENCY);
	  SpiritRadioSetPALeveldBm(POWER_INDEX_433, POWER_DBM_433);
	  SpiritRadioSetPALevelMaxIndex(POWER_INDEX_433);

	  SpiritRadioInit(&x_radio_init);

	  /* Configures the SPIRIT1 packet handler part*/
	  PktBasicInit x_basic_init = {
			  PREAMBLE_LENGTH,
			  SYNC_LENGTH,
			  SYNC_WORD,
			  LENGTH_TYPE,
			  LENGTH_WIDTH,
			  CRC_MODE,
			  CONTROL_LENGTH,
			  EN_ADDRESS,
			  EN_FEC,
			  EN_WHITENING
	  };
	  SpiritPktBasicInit(&x_basic_init);

	  /* FIFO */
	  SpiritLinearFifoSetAlmostFullThresholdRx(RX_FIFO_ALMOST_FULL_BYTES);
	  SpiritLinearFifoSetAlmostEmptyThresholdTx(TX_FIFO_ALMOST_EMPTY_BYTES);

	  /* Enable the following interrupt sources, routed to GPIO */
	  Spirit1GpioIrqInit(&xGpioIRQ);

	  SpiritIrqDeInit(NULL);
	  SpiritIrqClearStatus();

	  /*
	   * XXX-rrodriguezz 21/09/2016 Enable TX and RX, FIFO and sync interrupt conditions.
	   */
	  SpiritIrq(VALID_SYNC, S_ENABLE);
	  SpiritIrq(TX_DATA_SENT, S_ENABLE);
	  SpiritIrq(RX_DATA_READY, S_ENABLE);
	  SpiritIrq(TX_FIFO_ALMOST_EMPTY, S_ENABLE);
	  SpiritIrq(RX_FIFO_ALMOST_FULL, S_ENABLE);
	  SpiritIrq(TX_FIFO_ERROR, S_ENABLE);
	  SpiritIrq(RX_FIFO_ERROR, S_ENABLE);

	  /* Configure Spirit1 */
	  SpiritRadioPersistenRx(S_ENABLE);
	  SpiritQiSetSqiThreshold(SQI_TH_0);
	  SpiritQiSqiCheck(S_ENABLE);
	  SpiritQiSetRssiThresholddBm(CCA_THRESHOLD);
	  SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);
	  SET_INFINITE_RX_TIMEOUT();
	  SpiritRadioAFCFreezeOnSync(S_ENABLE);

	  /* Puts the SPIRIT1 in STANDBY mode (125us -> rx/tx) */
	  spirit1_strobe(SPIRIT1_STROBE_STANDBY);
	  spirit_on = OFF;
	  CLEAR_RXBUF();
//	  CLEAR_TXBUF();

	  /* Initializes the mcu pin as input, used for IRQ */
	  GPIO_InitStruct.Pin = SPIRIT_433_GPIO_IRQ;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(SPIRIT_433_GPIO_IRQ_PORT, &GPIO_InitStruct);

	  SpiritRadioSetPALeveldBm(POWER_INDEX_433, POWER_DBM_433);
	  SpiritRadioSetPALevelMaxIndex(POWER_INDEX_433);

	  osThreadDef(spirit1Task, spirit1Task_function, osPriorityRealtime, 0, 320);
	  spirit1TaskId = osThreadCreate(osThread(spirit1Task), NULL);

	  IRQ_ENABLE_433();

	  PRINTF("Spirit1 init done\n");
  }

  if(current_radio_driver == RADIO_SPIRIT_868){
	  HAL_GPIO_WritePin(SPIRIT_868_GPIO_SDN_PORT, SPIRIT_868_GPIO_SDN_PIN, GPIO_PIN_RESET);
	  osDelay(1);

	  /* Soft reset of core */
	  spirit1_strobe(SPIRIT1_STROBE_SRES);
	  osDelay(5);
	  /* Configures the SPIRIT1 radio part */
	  SRadioInit x_radio_init = {
			  /* XTAL_FREQUENCY, */
			  XTAL_OFFSET_PPM_868,
			  BASE_FREQUENCY_868,
			  CHANNEL_SPACE_868,
			  CHANNEL_NUMBER_868,
			  MODULATION_SELECT_868,
			  DATARATE_868,
			  FREQ_DEVIATION_868,
			  BANDWIDTH_868
	  };

	  SpiritRadioSetXtalFrequency(XTAL_FREQUENCY);
	  SpiritManagementSetFrequencyBase(XTAL_FREQUENCY);
	  SpiritRadioSetPALeveldBm(POWER_INDEX_868, POWER_DBM_868);
	  SpiritRadioSetPALevelMaxIndex(POWER_INDEX_868);

	  SpiritRadioInit(&x_radio_init);

	  /* Configures the SPIRIT1 packet handler part*/
	  PktBasicInit x_basic_init = {
			  PREAMBLE_LENGTH,
			  SYNC_LENGTH,
			  SYNC_WORD,
			  LENGTH_TYPE,
			  LENGTH_WIDTH,
			  CRC_MODE,
			  CONTROL_LENGTH,
			  EN_ADDRESS,
			  EN_FEC,
			  EN_WHITENING
	  };
	  SpiritPktBasicInit(&x_basic_init);

	  /* FIFO */
	  SpiritLinearFifoSetAlmostFullThresholdRx(RX_FIFO_ALMOST_FULL_BYTES);
	  SpiritLinearFifoSetAlmostEmptyThresholdTx(TX_FIFO_ALMOST_EMPTY_BYTES);
	  /* Enable the following interrupt sources, routed to GPIO */
	  Spirit1GpioIrqInit(&xGpioIRQ);

	  SpiritIrqDeInit(NULL);
	  SpiritIrqClearStatus();

	  /*
	   * XXX-rrodriguezz 21/09/2016 Enable TX and RX, FIFO and sync interrupt conditions.
	   */
	  SpiritIrq(VALID_SYNC, S_ENABLE);
	  SpiritIrq(TX_DATA_SENT, S_ENABLE);
	  SpiritIrq(RX_DATA_READY, S_ENABLE);
	  SpiritIrq(TX_FIFO_ALMOST_EMPTY, S_ENABLE);
	  SpiritIrq(RX_FIFO_ALMOST_FULL, S_ENABLE);
	  SpiritIrq(TX_FIFO_ERROR, S_ENABLE);
	  SpiritIrq(RX_FIFO_ERROR, S_ENABLE);


	  /* Configure Spirit1 */
	  SpiritRadioPersistenRx(S_ENABLE);
	  SpiritQiSetSqiThreshold(SQI_TH_0);
	  SpiritQiSqiCheck(S_ENABLE);
	  SpiritQiSetRssiThresholddBm(CCA_THRESHOLD);
	  SpiritTimerSetRxTimeoutStopCondition(SQI_ABOVE_THRESHOLD);
	  SET_INFINITE_RX_TIMEOUT();
	  SpiritRadioAFCFreezeOnSync(S_ENABLE);

	  /* Puts the SPIRIT1 in STANDBY mode (125us -> rx/tx) */
	  spirit1_strobe(SPIRIT1_STROBE_STANDBY);
	  spirit_on = OFF;
	  CLEAR_RXBUF();
//	  CLEAR_TXBUF();

	  /* Initializes the mcu pin as input, used for IRQ */
	  GPIO_InitStruct.Pin = SPIRIT_868_GPIO_IRQ;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(SPIRIT_868_GPIO_IRQ_PORT, &GPIO_InitStruct);

	  SpiritRadioSetPALeveldBm(POWER_INDEX_868, POWER_DBM_868);
	  SpiritRadioSetPALevelMaxIndex(POWER_INDEX_868);

	  spirit1_strobe(SPIRIT1_STROBE_READY);

	  osThreadDef(spirit1Task, spirit1Task_function, osPriorityRealtime, 0, 320);
	  spirit1TaskId = osThreadCreate(osThread(spirit1Task), NULL);

	  IRQ_ENABLE_868();

	  PRINTF("Spirit1 init done\n");
  }


  rcv_new_packet = 1;
  prev_packet_not_processed = 0;
  flush_remaining_packet = 0;
  sent_done = 0;

  return 0;
}


/**
 * @brief spirit1 task
 * @param argument
 */
void spirit1Task_function(void const * argument){

	int len;

	spirit1TaskId = osThreadGetId();
	while(1) {
		osThreadSuspend(spirit1TaskId);

		uint8_t data_ready = interrupt_thread_routine();

		while(data_ready){

#if USE_WOR
			if (RSOSC_disabled == 1)
			{
				cc1101_RCOSC_CALIBRATE();
			}
#endif /* USE_WOR */

			packetbuf_clear();
			len = spirit_radio_read(packetbuf_dataptr(), PACKETBUF_SIZE);

			if(len > 0) {

#if NULLRDC_CONF_802154_AUTOACK
				/* Check if the packet has an ACK request */
				frame802154_t info154;
				if(len > ACK_LEN &&
						frame802154_parse((char *)packetbuf_dataptr(), len, &info154) != 0) {
					if(info154.fcf.frame_type == FRAME802154_DATAFRAME &&
							info154.fcf.ack_required != 0 &&
							linkaddr_cmp((linkaddr_t *)&info154.dest_addr,
									&linkaddr_node_addr)) {

#if !XXX_ACK_WORKAROUND
						/* Send an ACK packet */
						uint8_t ack_frame[ACK_LEN] = {
								FRAME802154_ACKFRAME,
								0x00,
								info154.seq
						};
						IRQ_DISABLE();
						spirit1_strobe(SPIRIT1_STROBE_FTX);
						st_lib_spirit_pkt_basic_set_payload_length((uint16_t)ACK_LEN);
						st_lib_spirit_spi_write_linear_fifo((uint16_t)ACK_LEN, (uint8_t *)ack_frame);

						spirit_set_ready_state();
						IRQ_ENABLE();
						spirit1_strobe(SPIRIT1_STROBE_TX);
						BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_TX, 1 * SYSTEM_CONF_SECOND / 1000);
						BUSYWAIT_UNTIL(SPIRIT1_STATUS() != SPIRIT1_STATE_TX, 1 * SYSTEM_CONF_SECOND / 1000);
						ACKPRINTF("debug_ack: sent ack %d\n", ack_frame[2]);
#endif /* !XXX_ACK_WORKAROUND */
					}
				}
#endif /* NULLRDC_CONF_802154_AUTOACK */

				packetbuf_set_datalen(len);
				NETSTACK_RDC.input();
			}

			prev_packet_not_processed = 0;
			data_ready = interrupt_thread_routine();

	//		if(!IS_RXBUF_EMPTY()) {
	//			osThreadResume (spirit1TaskId);
	//		}

	//		if(interrupt_callback_wants_poll) {
	//			spirit1_interrupt_callback();
	//
	//			if(SPIRIT1_STATUS() == SPIRIT1_STATE_READY) {
	//				spirit1_strobe(SPIRIT1_STROBE_RX);
	//				BUSYWAIT_UNTIL((SPIRIT1_STATUS() == SPIRIT1_STATE_RX), 1 * SYSTEM_CONF_SECOND / 1000);
	//			}
	//		}
		}
	}
}


/**
 * @brief prepare a packet
 * @param payload the data payload
 * @param payload_len the payload lenght
 * @return RADIO_TX_OK if success. RADIO_TX_ERR if payload_len is not supported
 */
static int
spirit_radio_prepare(const void *payload, unsigned short payload_len)
{
  PRINTF("Spirit1: prep %u\n", payload_len);
  packet_is_prepared = 0;

  /* Checks if the payload length is supported */
  if(payload_len > MAX_PACKET_LEN) {
    return RADIO_TX_ERR;
  }

  spirit_txbuf =(uint8_t*) payload;
  remaining_tx_payload_bytes = payload_len;

  /* Should we delay for an ack? */
#if NULLRDC_CONF_802154_AUTOACK
  frame802154_t info154;
  wants_an_ack = 0;
  if(payload_len > ACK_LEN
     && frame802154_parse((char *)payload, payload_len, &info154) != 0) {
    if(info154.fcf.frame_type == FRAME802154_DATAFRAME
       && info154.fcf.ack_required != 0) {
      wants_an_ack = 1;
    }
  }
#endif /* NULLRDC_CONF_802154_AUTOACK */

  /* Sets the length of the packet to send */
  if(current_radio_driver == RADIO_SPIRIT_433){
	IRQ_DISABLE_433();
  }
  else if(current_radio_driver == RADIO_SPIRIT_868){
	IRQ_DISABLE_868();
  }

  spirit1_strobe(SPIRIT1_STROBE_FTX);

  SpiritPktBasicSetPayloadLength(payload_len);

  if (payload_len <= SPIRIT_MAX_FIFO_LEN){
	  SpiritSpiWriteLinearFifo(payload_len, (uint8_t *)payload);
	  remaining_tx_payload_bytes = 0;
  }
  else{
	  SpiritSpiWriteLinearFifo(SPIRIT_MAX_FIFO_LEN, (uint8_t *)payload);
	  spirit_txbuf += SPIRIT_MAX_FIFO_LEN;
	  remaining_tx_payload_bytes = remaining_tx_payload_bytes - SPIRIT_MAX_FIFO_LEN;
  }

  if(current_radio_driver == RADIO_SPIRIT_433){
	IRQ_ENABLE_433();
  }
  else if(current_radio_driver == RADIO_SPIRIT_868){
	IRQ_ENABLE_868();
  }


  PRINTF("PREPARE OUT\n");

  packet_is_prepared = 1;
  return RADIO_TX_OK;
}

/**
 * @brief transmit a packer
 * @param payload_len the payload lenght
 * @return RADIO_TX_OK if success. RADIO_TX_ERR if packet is not prepare
 */
static int
spirit_radio_transmit(unsigned short payload_len)
{
  /* This function blocks until the packet has been transmitted */

  PRINTF("TRANSMIT IN\n");
  if(!packet_is_prepared) {
    return RADIO_TX_ERR;
  }
  if (receiving_packet){
	  return RADIO_TX_ERR;
  }

  /* Puts the SPIRIT1 in TX state */
  sent_done = 0;

  if (receiving_packet){
	  return RADIO_TX_ERR;
  }

  spirit_set_ready_state();
  just_got_an_ack = 0;

//  BUSYWAIT_UNTIL((!receiving_packet), 100 * SYSTEM_CONF_SECOND / 1000);	//Wait if it is receiving data
  while(receiving_packet);
  if(receiving_packet){
	  leds_on(LEDS_RED2);					//This should never happen
	  while(1);
  }

  spirit1_strobe(SPIRIT1_STROBE_TX);
  BUSYWAIT_UNTIL((SPIRIT1_STATUS() == SPIRIT1_STATE_TX), 1 * SYSTEM_CONF_SECOND / 1000);
  while((SPIRIT1_STATUS() != SPIRIT1_STATE_TX));

  BUSYWAIT_UNTIL(sent_done, 1000 * SYSTEM_CONF_SECOND / 1000); //RRZ Don't call SPIRIT1_STATUS fos waiting TX complete as it uses SPI, and it may also be used in interrupt routines, which may cause errors

  //En este momento el interfaz radio esta en RX. SE pone asi en la interrupcion al terminar de enviar un paquete

  /* Reset radio - needed for immediate RX of ack */
//  CLEAR_TXBUF();
  CLEAR_RXBUF();
  if(current_radio_driver == RADIO_SPIRIT_433){
	IRQ_DISABLE_433();
  }
  else if(current_radio_driver == RADIO_SPIRIT_868){
	IRQ_DISABLE_868();
  }

  SpiritIrqClearStatus();


  if(current_radio_driver == RADIO_SPIRIT_433){
	IRQ_ENABLE_433();
  }
  else if(current_radio_driver == RADIO_SPIRIT_868){
	IRQ_ENABLE_868();
  }



#if XXX_ACK_WORKAROUND
  just_got_an_ack = 1;
#endif /* XXX_ACK_WORKAROUND */

#if NULLRDC_CONF_802154_AUTOACK
  if(wants_an_ack) {
    rtimer_txdone = OS_GET_TIME();
    BUSYWAIT_UNTIL(just_got_an_ack, 2 * SYSTEM_CONF_SECOND / 1000);
    rtimer_rxack = OS_GET_TIME();

    if(just_got_an_ack) {
      ACKPRINTF("debug_ack: ack received after %u/%u ticks\n",
                (uint32_t)(rtimer_rxack - rtimer_txdone), 2 * SYSTEM_CONF_SECOND / 1000);
    } else {
      ACKPRINTF("debug_ack: no ack received\n");
    }
  }
#endif /* NULLRDC_CONF_802154_AUTOACK */

  PRINTF("TRANSMIT OUT\n");

//  CLEAR_TXBUF();

  packet_is_prepared = 0;

//  osDelay(1);

  return RADIO_TX_OK;
}

/**
 * @brief prepare and transmit a packet
 * @param payload the data payload
 * @param payload_len the payload lenght
 * @return RADIO_TX_OK if success. RADIO_TX_ERR if packet is not prepare
 */
static int
spirit_radio_send(const void *payload, unsigned short payload_len)
{
  if(spirit_radio_prepare(payload, payload_len) == RADIO_TX_ERR) {
    return RADIO_TX_ERR;
  }
  return spirit_radio_transmit(payload_len);
}


/**
 * @brief read a packet
 * @param buf rx buffer
 * @param bufsize size of rx buffer
 * @return 0 if rxbuf is empty or buf has an incorrect size. bufsize if success
 */
static int
spirit_radio_read(void *buf, unsigned short bufsize)
{
  PRINTF("READ IN\n");

  if(bufsize < rcv_packet_len) {
    /* If buf has the correct size */
    PRINTF("TOO SMALL BUF\n");
    return 0;
  } else {
    /* Copies the packet received */
    memcpy(buf, rxbuf, rcv_packet_len);

    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, last_rssi);        /* MGR */
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, last_lqi); /* MGR */
    bufsize = rcv_packet_len;

    CLEAR_RXBUF();

    PRINTF("READ OUT\n");

    return bufsize;
  }
}

/**
 * @brief check if a channel is clear
 * @return 0 if is clear. 1 if not.
 */
static int
spirit_radio_channel_clear(void)
{
	float rssi_value;
	/* Local variable used to memorize the SPIRIT1 state */
	uint8_t spirit_state = ON;

	PRINTF("CHANNEL CLEAR IN\n");

	if(spirit_on == OFF) {
		/* Wakes up the SPIRIT1 */
		spirit_radio_on();
		spirit_state = OFF;
	}

	/* HARDWARE DEPENDENT */
	if(current_radio_driver == RADIO_SPIRIT_433){
		IRQ_DISABLE_433();
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		IRQ_DISABLE_868();
	}

	/* HARDWARE DEPENDENT */
	spirit1_strobe(SPIRIT1_STROBE_SABORT);
	/*  SpiritCmdStrobeSabort();*/
	SpiritIrqClearStatus();

	if(current_radio_driver == RADIO_SPIRIT_433){
		IRQ_ENABLE_433();
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		IRQ_ENABLE_868();
	}

	{
		uint32_t timeout = OS_GET_TIME() + 5 * SYSTEM_CONF_SECOND / 1000;
		do {
			SpiritRefreshStatus();
		} while((g_xStatus.MC_STATE != MC_STATE_READY) && (OS_GET_TIME() < timeout));
		if(OS_GET_TIME() < timeout) {
			return 1;
		}
	}

	/* Stores the RSSI value */
	rssi_value = SpiritQiGetRssidBm();

	/* Puts the SPIRIT1 in its previous state */
	if(spirit_state == OFF) {
		spirit_radio_off();
	} else {
		spirit1_strobe(SPIRIT1_STROBE_RX);
		/*    SpiritCmdStrobeRx();*/
		BUSYWAIT_UNTIL((SPIRIT1_STATUS() == SPIRIT1_STATE_RX), 5 * SYSTEM_CONF_SECOND / 1000);
	}

	PRINTF("CHANNEL CLEAR OUT\n");

	/* Checks the RSSI value with the threshold */
	if(rssi_value < CCA_THRESHOLD) {
		return 0;
	} else {
		return 1;
	}
}

/**
 * @brief checks if sprit1 is receiving a packet
 * @return 1 if is receiving a packet
 */
static int
spirit_radio_receiving_packet(void)
{
  return receiving_packet;
}

/**
 * @brief ckecks if a packet is pending
 * @return 1 if RXBUF is not empty.
 */
static int
spirit_radio_pending_packet(void)
{
  PRINTF("PENDING PACKET\n");
  return prev_packet_not_processed;
}

/**
 * @brief switch off spirit1 radio
 * @return 0 if success, 1 if fails
 */
static int
spirit_radio_off(void)
{

	PRINTF("Spirit1: ->off\n");
	if(spirit_on == ON) {
		/* Disables the mcu to get IRQ from the SPIRIT1 */
		if(current_radio_driver == RADIO_SPIRIT_433){
			IRQ_DISABLE_433();
		}
		else if(current_radio_driver == RADIO_SPIRIT_868){
			IRQ_DISABLE_868();
		}
		/* first stop rx/tx */
		spirit1_strobe(SPIRIT1_STROBE_SABORT);
		/* Clear any pending irqs */
		SpiritIrqClearStatus();
		osDelay(5);
		//BUSYWAIT_UNTIL((SPIRIT1_STATUS() == SPIRIT1_STATE_READY), 5 * SYSTEM_CONF_SECOND / 1000);
		if(SPIRIT1_STATUS() != SPIRIT1_STATE_READY) {
			leds_on(LEDS_RED2);
			while(1);
			PRINTF("Spirit1: failed off->ready\n");
			return 1;
		}

		/* Puts the SPIRIT1 in STANDBY */
		spirit1_strobe(SPIRIT1_STROBE_STANDBY);
		osDelay(5);
		//BUSYWAIT_UNTIL((SPIRIT1_STATUS() == SPIRIT1_STATE_STANDBY), 5 * SYSTEM_CONF_SECOND / 1000);
		if(SPIRIT1_STATUS() != SPIRIT1_STATE_STANDBY) {
			leds_on(LEDS_RED2);
			while(1);
			PRINTF("Spirit1: failed off->stdby\n");
			return 1;
		}

		spirit_on = OFF;
//		CLEAR_TXBUF();
		CLEAR_RXBUF();
	}
	PRINTF("Spirit1: off.\n");


	return 0;
}

/**
 * @brief switch on spirit1 radio
 * @return 0 if success, 1 if fails
 */
static int
spirit_radio_on(void)
{
	SpiritStatus status;
	uint8_t tempRegValue;
	status = RadioSpiReadRegisters(MC_STATE1_BASE, 1, &tempRegValue);
	PRINTF("Spirit1: on\n");
	spirit1_strobe(SPIRIT1_STROBE_SABORT);
	osDelay(1);
	if(spirit_on == OFF) {
		if(current_radio_driver == RADIO_SPIRIT_433){
			IRQ_DISABLE_433();
		}
		else if(current_radio_driver == RADIO_SPIRIT_868){
			IRQ_DISABLE_868();
		}

		/* ensure we are in READY state as we go from there to Rx */
		spirit1_strobe(SPIRIT1_STROBE_FRX);
		spirit1_strobe(SPIRIT1_STROBE_READY);
		osDelay(1);
		status = RadioSpiReadRegisters(MC_STATE1_BASE, 1, &tempRegValue);
		if(status.MC_STATE != MC_STATE_READY) {
			leds_on(LEDS_RED2);
			PRINTF("Spirit1: failed to turn on\n");
			while(1) ;
			/* return 1; */
		}

		/* now we go to Rx */
		spirit1_strobe(SPIRIT1_STROBE_RX);
		osDelay(1);
		//    BUSYWAIT_UNTIL(SPIRIT1_STATUS() == SPIRIT1_STATE_RX, 5 * SYSTEM_CONF_SECOND / 1000);
		if(SPIRIT1_STATUS() != SPIRIT1_STATE_RX) {
			leds_on(LEDS_RED2);
			PRINTF("Spirit1: failed to enter rx\n");
			while(1) ;
			/* return 1; */
		}

		/* Enables the mcu to get IRQ from the SPIRIT1 */
		if(current_radio_driver == RADIO_SPIRIT_433){
			IRQ_ENABLE_433();
		}
		else if(current_radio_driver == RADIO_SPIRIT_868){
			IRQ_ENABLE_868();
		}

		spirit_on = ON;
	}

	return 0;
}

static int spirit_radio_close(void){

	receiving_packet = 0;
	prev_packet_not_processed = 0;
	flush_remaining_packet = 0;
	rcv_new_packet = 1;

	osTimerStop(receive_timer_id);
	osTimerDelete(receive_timer_id);

	/* Soft reset of core */
	if(current_radio_driver == RADIO_SPIRIT_433){
		IRQ_DISABLE_433();
		__HAL_GPIO_EXTI_CLEAR_IT(SPIRIT_433_GPIO_IRQ);

		spirit1_strobe(SPIRIT1_STROBE_SRES);
		osDelay(5);

		RadioSpiClose();		//Close spi port

		//Shutdown the device
		HAL_GPIO_WritePin(SPIRIT_433_GPIO_SDN_PORT, SPIRIT_433_GPIO_SDN_PIN, GPIO_PIN_SET);

		CLEAR_RXBUF();
//		CLEAR_TXBUF();


		osThreadTerminate(spirit1TaskId);
	}
	else if(current_radio_driver == RADIO_SPIRIT_868){
		IRQ_DISABLE_868();
		__HAL_GPIO_EXTI_CLEAR_IT(SPIRIT_868_GPIO_IRQ);

		spirit1_strobe(SPIRIT1_STROBE_SRES);
		osDelay(5);

		RadioSpiClose();		//Close spi port
		//Shutdown the device
		HAL_GPIO_WritePin(SPIRIT_868_GPIO_SDN_PORT, SPIRIT_868_GPIO_SDN_PIN, GPIO_PIN_SET);

		CLEAR_RXBUF();
//		CLEAR_TXBUF();


		osThreadTerminate(spirit1TaskId);
	}
	spirit_on = OFF;

	return 0;
}

uint8_t interrupt_thread_routine(void){
#define INTPRINTF(...) /* PRINTF */
	SpiritIrqs x_irq_status;
//	if(current_radio_driver == RADIO_SPIRIT_433){
//	  if(spirit_spi_busy_433() || interrupt_callback_in_progress) {
//		osThreadResume (spirit1TaskId);
//		interrupt_callback_wants_poll = 1;
//		return;
//	  }
//	}
//	else if(current_radio_driver == RADIO_SPIRIT_868){
//	  if(spirit_spi_busy_868() || interrupt_callback_in_progress) {
//		osThreadResume (spirit1TaskId);
//		interrupt_callback_wants_poll = 1;
//		return;
//	  }
//	}

	uint8_t data_received = 0;
  interrupt_callback_wants_poll = 0;
  interrupt_callback_in_progress = 1;
  /* get interrupt source from radio */

  SpiritIrqGetStatus(&x_irq_status);
  SpiritIrqClearStatus();

  if(x_irq_status.IRQ_RX_FIFO_ERROR) {
    receiving_packet = 0;
    interrupt_callback_in_progress = 0;
    spirit1_strobe(SPIRIT1_STROBE_FRX);
    return data_received;
  }

  if(x_irq_status.IRQ_TX_FIFO_ERROR) {
    receiving_packet = 0;
    interrupt_callback_in_progress = 0;
    spirit1_strobe(SPIRIT1_STROBE_FTX);
    return data_received;
  }

  /* The IRQ_VALID_SYNC is used to notify a new packet is coming */
  if(x_irq_status.IRQ_VALID_SYNC) {
    INTPRINTF("SYNC\n");
    receiving_packet = 1;
    osTimerStart (receive_timer_id, RECEIVE_TIMEOUT); 		// start timer. If the packet has not been received in this time, it is discarted
    return data_received;
  }



  /* The IRQ_TX_FIFO_ALMOST_EMPTY notifies the TX FIFO is getting empty. Refill it with the remaining packet */
  while(x_irq_status.IRQ_TX_FIFO_ALMOST_EMPTY) {
    INTPRINTF("TX_FIFO_EMPTY\n");

	if(remaining_tx_payload_bytes > 0){
		if(remaining_tx_payload_bytes <= (SPIRIT_MAX_FIFO_LEN-TX_FIFO_ALMOST_EMPTY_BYTES)){
		  SpiritSpiWriteLinearFifo(remaining_tx_payload_bytes, (uint8_t *)spirit_txbuf);
		  remaining_tx_payload_bytes = 0;
		}
		else{
		  SpiritSpiWriteLinearFifo((SPIRIT_MAX_FIFO_LEN-TX_FIFO_ALMOST_EMPTY_BYTES), (uint8_t *)spirit_txbuf);
		  spirit_txbuf += (SPIRIT_MAX_FIFO_LEN-TX_FIFO_ALMOST_EMPTY_BYTES);
		  remaining_tx_payload_bytes = remaining_tx_payload_bytes - (SPIRIT_MAX_FIFO_LEN-TX_FIFO_ALMOST_EMPTY_BYTES);
		}
    }
//	SPIRIT1_STATUS();		//RRZ Por algun motivo que no entiendo hace falta añadir esto para que funcione. Si no a veces falla la transmision de paquetes largos
	SpiritIrqGetStatus(&x_irq_status);
	SpiritIrqClearStatus();
//    return;
  }


  /* The IRQ_TX_DATA_SENT notifies the packet received. Puts the SPIRIT1 in RX */
  if(x_irq_status.IRQ_TX_DATA_SENT) {
    spirit1_strobe(SPIRIT1_STROBE_RX);
    INTPRINTF("SENT\n");
    interrupt_callback_in_progress = 0;
    sent_done = 1;
    return data_received;
  }

  /* The IRQ_RX_FIFO_ALMOST_FULL notifies the RX FIFO is getting full. Start receiving the packet */
  if(x_irq_status.IRQ_RX_FIFO_ALMOST_FULL) {
    INTPRINTF("RX_FIFO_FULL\n");

    if(prev_packet_not_processed || flush_remaining_packet){				//A previous received packet has not been processed by the thread
    	spirit1_strobe(SPIRIT1_STROBE_FRX);									//Flush this packet
    	flush_remaining_packet = 1;
    	return data_received;
    }

    if(rcv_new_packet){				//The received packet is new
    	rcv_new_packet = 0;
    	current_rx_bytes = 0;

    }

	uint16_t bytes_in_fifo;
	while((bytes_in_fifo = SpiritLinearFifoReadNumElementsRxFifo())>0){
		if( (bytes_in_fifo+current_rx_bytes) <= MAX_PACKET_LEN){
			SpiritSpiReadLinearFifo(bytes_in_fifo, rxbuf+(current_rx_bytes));
			current_rx_bytes += bytes_in_fifo;
		}
	}
	SpiritIrqGetStatus(&x_irq_status);
	SpiritIrqClearStatus();


  }
  /* The IRQ_RX_DATA_READY notifies a new packet arrived */
   if(x_irq_status.IRQ_RX_DATA_READY) {

	    if(prev_packet_not_processed | flush_remaining_packet){				//A previous received packet has not been processed by the thread
	    	spirit1_strobe(SPIRIT1_STROBE_FRX);		//Flush this packet
	    	flush_remaining_packet = 0;
//	    	goto CHECK_IRQ;
	    	return data_received;
	    }


	    rcv_packet_len = SpiritPktBasicGetReceivedPktLength();


	    if(rcv_new_packet){				//The received packet is new

	    	rcv_new_packet = 0;
	    	current_rx_bytes = 0;

	    	uint16_t bytes_in_fifo = SpiritLinearFifoReadNumElementsRxFifo();

			if( (bytes_in_fifo) < MAX_PACKET_LEN){
				SpiritSpiReadLinearFifo(bytes_in_fifo, rxbuf);
			}
		}
	    else{
	    	uint16_t bytes_in_fifo;
			if((bytes_in_fifo = SpiritLinearFifoReadNumElementsRxFifo())>0){
				if( (bytes_in_fifo+current_rx_bytes) <= MAX_PACKET_LEN){
					SpiritSpiReadLinearFifo(bytes_in_fifo, rxbuf+(current_rx_bytes));
					current_rx_bytes += bytes_in_fifo;
				}
			}
	    }

		spirit1_strobe(SPIRIT1_STROBE_FRX);

		last_rssi = (packetbuf_attr_t)SpiritQiGetRssi();  /* MGR */
		last_lqi = (packetbuf_attr_t)SpiritQiGetSqi();    /* MGR */

		receiving_packet = 0;
		rcv_new_packet = 1;

		prev_packet_not_processed = 1;
//		osThreadResume (spirit1TaskId);
		data_received = 1;
		spirit1_strobe(SPIRIT1_STROBE_RX);		//Set to RX

// #if NULLRDC_CONF_802154_AUTOACK
//     if(spirit_rxbuf[0] == ACK_LEN) {
//       /* For debugging purposes we assume this is an ack for us */
//       just_got_an_ack = 1;
//     }
// #endif /* NULLRDC_CONF_802154_AUTOACK */
//
     interrupt_callback_in_progress = 0;
     return data_received;
   }

   if(x_irq_status.IRQ_RX_DATA_DISC) {
      /* RX command - to ensure the device will be ready for the next reception */
      if(x_irq_status.IRQ_RX_TIMEOUT) {
      	SpiritCmdStrobeFlushRxFifo();
      	rx_timeout = SET;
      }
    }

    interrupt_callback_in_progress = 0;
    return data_received;
}


/**
 * @brief spirit1 interrupt callback
 */
void spirit1_interrupt_callback(void)
{
	if(spirit1TaskId != NULL){
		osThreadResume (spirit1TaskId);
	}
}

void receive_timeout  (void const *arg){
	receiving_packet = 0;
	prev_packet_not_processed = 0;
	flush_remaining_packet = 0;
	rcv_new_packet = 1;
	spirit1_strobe(SPIRIT1_STROBE_FRX);
}

/*---------------------------------------------------------------------------*/
