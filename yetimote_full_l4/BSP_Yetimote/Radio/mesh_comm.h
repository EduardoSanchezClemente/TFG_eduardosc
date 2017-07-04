/*
 * Copyright (c) 2016, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * mesh_comm.h
 *
 *  Created on: 9/5/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 *
 */
/**
 * @file mesh_comm.h
 */

#ifndef APPLICATION_USER_BSP_YETIMOTE_RADIO_MESH_COMM_H_
#define APPLICATION_USER_BSP_YETIMOTE_RADIO_MESH_COMM_H_

#include "yetimote-conf.h"
#include "rime.h"
#include "linkaddr.h"
#include "mesh.h"
#include "generic_list.h"
#include "vcom_usb_inout.h"
#include "commands.h"
#include <unistd.h>
#include <string.h>

typedef struct mesh_comm_{
	gen_list* meshList;
	osMutexId meshList_mutex;
}mesh_comm_t;

mesh_comm_t* new_mesh_comm(void);
retval_t delete_mesh_comm(mesh_comm_t* msh_comm);

retval_t yetimote_mesh_open(mesh_comm_t* msh_comm, uint16_t channel, void (*rcv_callback_func)(struct mesh_conn *, const linkaddr_t *, uint8_t));
retval_t yetimote_mesh_close(mesh_comm_t* msh_comm, uint16_t channel);
retval_t yetimote_mesh_send(mesh_comm_t* msh_comm, uint16_t channel, linkaddr_t addr, uint8_t* payload, uint16_t payload_len);

#endif /* APPLICATION_USER_BSP_YETIMOTE_RADIO_MESH_COMM_H_ */
