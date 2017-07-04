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
 * generic_list.c
 *
 *  Created on: 29/3/2016
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@die.upm.es>
 */

#include "generic_list.h"



gen_list* gen_list_init(){

	gen_list* list;

	list = pvPortMalloc(sizeof(gen_list));
	list->item = NULL;
	list->next = NULL;
	return list;
}

void gen_list_remove_all(gen_list* list){

	while(list->next != NULL){
		gen_list_remove_last(list);
	}
	vPortFree(list);
}


void gen_list_add(gen_list* list, void* item){

	gen_list* current = list;

	while(current->next != NULL){
		current = current->next;
	}
	current->next = pvPortMalloc(sizeof(gen_list));
	current = current->next;
	current->item = item;
	current->next = NULL;
}

void gen_list_remove_last(gen_list* list){

	if(list->next == NULL){			//Esto significa que la lista esta vacia, ya que el primer miembro se usa de referencia.
		return;
	}

	gen_list* current = list;

	while(current->next->next != NULL){
		current = current->next;
	}

	vPortFree(current->next->item);
	vPortFree(current->next);
	current->next = NULL;

}

void gen_list_remove(gen_list* list, void* item){

	gen_list* current = list;


	while(current->next != NULL){

		if(current->next->item == item){
			vPortFree(current->next->item);
			gen_list* next = current->next->next;
			vPortFree(current->next);
			current->next = next;
		}

		else{
			current = current->next;
		}

	}
}


void* gen_list_get_last(gen_list* list){

	if(list->next == NULL){			//Esto significa que la lista esta vacia, ya que el primer miembro se usa de referencia.
		return NULL;
	}

	gen_list* current = list;

	while(current->next->next != NULL){
		current = current->next;
	}

	return (void*) current->next->item;
}
