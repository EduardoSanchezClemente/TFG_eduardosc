/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Rime buffer (packetbuf) management
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

/**
 * \addtogroup packetbuf
 * @{
 */

#include <string.h>

#include "packetbuf.h"


osMutexId packetbuf_mutex;
osMutexDef(packetbuf_mutex);


struct packetbuf_attr packetbuf_attrs[PACKETBUF_NUM_ATTRS];
struct packetbuf_addr packetbuf_addrs[PACKETBUF_NUM_ADDRS];


static uint16_t buflen, bufptr;
static uint8_t hdrptr;

/* The declarations below ensure that the packet buffer is aligned on
   an even 32-bit boundary. On some platforms (most notably the
   msp430 or OpenRISC), having a potentially misaligned packet buffer may lead to
   problems when accessing words. */
static uint32_t packetbuf_aligned[(PACKETBUF_SIZE + PACKETBUF_HDR_SIZE + 3) / 4];
static uint8_t *packetbuf = (uint8_t *)packetbuf_aligned;

static uint8_t *packetbufptr;

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


/*---------------------------------------------------------------------------*/
void
packetbuf_init(void)
{	//Create the mutex used to protect the packetbuf

	packetbuf_mutex = osMutexCreate(osMutex(packetbuf_mutex));

	packetbuf_clear();
}
/*---------------------------------------------------------------------------*/
void
packetbuf_clear(void)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	buflen = bufptr = 0;
	hdrptr = PACKETBUF_HDR_SIZE;
	packetbufptr = &packetbuf[PACKETBUF_HDR_SIZE];
	osMutexRelease(packetbuf_mutex);
	packetbuf_attr_clear();
}
/*---------------------------------------------------------------------------*/
void
packetbuf_clear_hdr(void)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	hdrptr = PACKETBUF_HDR_SIZE;
	osMutexRelease(packetbuf_mutex);
}
/*---------------------------------------------------------------------------*/
int
packetbuf_copyfrom(const void *from, uint16_t len)
{
  uint16_t l;

  packetbuf_clear();
  osMutexWait(packetbuf_mutex, osWaitForever);
  l = len > PACKETBUF_SIZE? PACKETBUF_SIZE: len;
  memcpy(packetbufptr, from, l);
  buflen = l;
  osMutexRelease(packetbuf_mutex);
  return l;
}
/*---------------------------------------------------------------------------*/
void
packetbuf_compact(void)
{
  int i, len;
  osMutexWait(packetbuf_mutex, osWaitForever);
  if(bufptr > 0) {
	  	osMutexRelease(packetbuf_mutex);
		len = packetbuf_datalen() + PACKETBUF_HDR_SIZE;
		osMutexWait(packetbuf_mutex, osWaitForever);
		for(i = PACKETBUF_HDR_SIZE; i < len; i++) {
		packetbuf[i] = packetbuf[bufptr + i];
		}

		bufptr = 0;
  }
  osMutexRelease(packetbuf_mutex);
}
/*---------------------------------------------------------------------------*/
int
packetbuf_copyto_hdr(uint8_t *to)
{
#if DEBUG_LEVEL > 0
  {
    int i;
    PRINTF("packetbuf_write_hdr: header:\n");
    for(i = hdrptr; i < PACKETBUF_HDR_SIZE; ++i) {
      PRINTF("0x%02x, ", packetbuf[i]);
    }
    PRINTF("\n");
  }
#endif /* DEBUG_LEVEL */
  osMutexWait(packetbuf_mutex, osWaitForever);
  memcpy(to, packetbuf + hdrptr, PACKETBUF_HDR_SIZE - hdrptr);
  osMutexRelease(packetbuf_mutex);
  return PACKETBUF_HDR_SIZE - hdrptr;
}
/*---------------------------------------------------------------------------*/
int
packetbuf_copyto(void *to)
{
#if DEBUG_LEVEL > 0
  {
    int i;
    char buffer[1000];
    char *bufferptr = buffer;
    int  bufferlen = 0;
    
    bufferptr = 0;
    for(i = hdrptr; i < PACKETBUF_HDR_SIZE; ++i) {
      bufferptr += sprintf(bufferptr, "0x%02x, ", packetbuf[i]);
    }
    PRINTF("packetbuf_write: header: %s\n", buffer);
    bufferptr = buffer;
    bufferptr = 0;
    for(i = bufptr; ((i < buflen + bufptr) && (bufferlen < (sizeof(buffer) - 10))); ++i) {
      bufferlen += sprintf(bufferptr + bufferlen, "0x%02x, ", packetbufptr[i]);
    }
    PRINTF("packetbuf_write: data: %s\n", buffer);
  }
#endif /* DEBUG_LEVEL */
  osMutexWait(packetbuf_mutex, osWaitForever);
  if(PACKETBUF_HDR_SIZE - hdrptr + buflen > PACKETBUF_SIZE) {
    /* Too large packet */
	  osMutexRelease(packetbuf_mutex);
	  return 0;
  }
  memcpy(to, packetbuf + hdrptr, PACKETBUF_HDR_SIZE - hdrptr);
  memcpy((uint8_t *)to + PACKETBUF_HDR_SIZE - hdrptr, packetbufptr + bufptr,
	 buflen);
  osMutexRelease(packetbuf_mutex);
  return PACKETBUF_HDR_SIZE - hdrptr + buflen;
}
/*---------------------------------------------------------------------------*/
int
packetbuf_hdralloc(int size)
{
	uint16_t totlen = packetbuf_totlen();
	osMutexWait(packetbuf_mutex, osWaitForever);
	if(hdrptr >= size && totlen + size <= PACKETBUF_SIZE) {
		hdrptr -= size;
		osMutexRelease(packetbuf_mutex);
		return 1;
	}
	osMutexRelease(packetbuf_mutex);
	return 0;
}
/*---------------------------------------------------------------------------*/
void
packetbuf_hdr_remove(int size)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	hdrptr += size;
	osMutexRelease(packetbuf_mutex);
}
/*---------------------------------------------------------------------------*/
int
packetbuf_hdrreduce(int size)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
  if(buflen < size) {
	osMutexRelease(packetbuf_mutex);
	return 0;
  }

  bufptr += size;
  buflen -= size;
  osMutexRelease(packetbuf_mutex);
  return 1;
}
/*---------------------------------------------------------------------------*/
void
packetbuf_set_datalen(uint16_t len)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	buflen = len;
	osMutexRelease(packetbuf_mutex);
}
/*---------------------------------------------------------------------------*/
void *
packetbuf_dataptr(void)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	void * pt = (void *)(&packetbuf[bufptr + PACKETBUF_HDR_SIZE]);
	osMutexRelease(packetbuf_mutex);
	return pt;
}
/*---------------------------------------------------------------------------*/
void *
packetbuf_hdrptr(void)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	void * pt = (void *)(&packetbuf[hdrptr]);
	osMutexRelease(packetbuf_mutex);
	return  pt;
}
/*---------------------------------------------------------------------------*/
uint16_t
packetbuf_datalen(void)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	uint16_t len = buflen;
	osMutexRelease(packetbuf_mutex);
	return len;
}
/*---------------------------------------------------------------------------*/
uint8_t
packetbuf_hdrlen(void)
{
  uint8_t hdrlen;

  osMutexWait(packetbuf_mutex, osWaitForever);
  hdrlen = PACKETBUF_HDR_SIZE - hdrptr;
  if(hdrlen) {
    /* outbound packet */
	  osMutexRelease(packetbuf_mutex);
	  return hdrlen;
  } else {
    /* inbound packet */
	  osMutexRelease(packetbuf_mutex);
	  return bufptr;
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
packetbuf_totlen(void)
{
  return packetbuf_hdrlen() + packetbuf_datalen();
}
/*---------------------------------------------------------------------------*/
void
packetbuf_attr_clear(void)
{
  int i;
  osMutexWait(packetbuf_mutex, osWaitForever);
  for(i = 0; i < PACKETBUF_NUM_ATTRS; ++i) {
    packetbuf_attrs[i].val = 0;
  }
  for(i = 0; i < PACKETBUF_NUM_ADDRS; ++i) {
    linkaddr_copy(&packetbuf_addrs[i].addr, &linkaddr_null);
  }
  osMutexRelease(packetbuf_mutex);
}
/*---------------------------------------------------------------------------*/
void
packetbuf_attr_copyto(struct packetbuf_attr *attrs,
		    struct packetbuf_addr *addrs)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	memcpy(attrs, packetbuf_attrs, sizeof(packetbuf_attrs));
	memcpy(addrs, packetbuf_addrs, sizeof(packetbuf_addrs));
	osMutexRelease(packetbuf_mutex);
}
/*---------------------------------------------------------------------------*/
void
packetbuf_attr_copyfrom(struct packetbuf_attr *attrs,
		      struct packetbuf_addr *addrs)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	memcpy(packetbuf_attrs, attrs, sizeof(packetbuf_attrs));
	memcpy(packetbuf_addrs, addrs, sizeof(packetbuf_addrs));
	osMutexRelease(packetbuf_mutex);
}
/*---------------------------------------------------------------------------*/
#if !PACKETBUF_CONF_ATTRS_INLINE
int
packetbuf_set_attr(uint8_t type, const packetbuf_attr_t val)
{
/*   packetbuf_attrs[type].type = type; */
	osMutexWait(packetbuf_mutex, osWaitForever);
	packetbuf_attrs[type].val = val;
	osMutexRelease(packetbuf_mutex);
	return 1;
}
/*---------------------------------------------------------------------------*/
packetbuf_attr_t
packetbuf_attr(uint8_t type)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	packetbuf_attr_t val = packetbuf_attrs[type].val;
	osMutexRelease(packetbuf_mutex);
	return val;
}
/*---------------------------------------------------------------------------*/
int
packetbuf_set_addr(uint8_t type, const linkaddr_t *addr)
{
/*   packetbuf_addrs[type - PACKETBUF_ADDR_FIRST].type = type; */
	osMutexWait(packetbuf_mutex, osWaitForever);
	linkaddr_copy(&packetbuf_addrs[type - PACKETBUF_ADDR_FIRST].addr, addr);
	osMutexRelease(packetbuf_mutex);
	return 1;
}
/*---------------------------------------------------------------------------*/
const linkaddr_t *
packetbuf_addr(uint8_t type)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	linkaddr_t* i = &packetbuf_addrs[type - PACKETBUF_ADDR_FIRST].addr;
	osMutexRelease(packetbuf_mutex);
	return i;
}
/*---------------------------------------------------------------------------*/
#endif /* PACKETBUF_CONF_ATTRS_INLINE */
int
packetbuf_holds_broadcast(void)
{
	osMutexWait(packetbuf_mutex, osWaitForever);
	int i = linkaddr_cmp(&packetbuf_addrs[PACKETBUF_ADDR_RECEIVER - PACKETBUF_ADDR_FIRST].addr, &linkaddr_null);
	osMutexRelease(packetbuf_mutex);
	return i;
}
/*---------------------------------------------------------------------------*/

/** @} */
