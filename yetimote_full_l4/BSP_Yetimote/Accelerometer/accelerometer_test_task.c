/*
 * Copyright (c) 2017, Universidad Politecnica de Madrid - B105 Electronic Systems Lab
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
 * accelerometer_test_task.c
 *
 *  Created on: 8 de feb. de 2017
 *      Author: Roberto Rodriguez-Zurrunero  <r.rodriguezz@b105.upm.es>
 *
 */
/**
 * @file accelerometer_test_task.c
 */

#include "accelerometer_driver.h"
#include "yetimote-conf.h"
#include "leds.h"
#include "output_packet.h"
#include "output_data.h"
#include "arm_math.h"
#include "time_control.h"

extern void accelerometer_test_task_callback(void const * argument);

extern time_control_t* time_control;
output_data_t* output_data;
output_packet_t* output_packet;

//accelerometer_driver_t* lis3dh_driver;
//accelerometer_driver_t* lis3dsh_driver;
accelerometer_driver_t* accelerometer_driver;

retval_t send_done(struct output_data_* output_data);

void accelerometer_test_task_callback(void const * argument){

	uint16_t i;

//	lis3dh_driver = new_accelerometer_driver(LIS3DH);
	accelerometer_driver = new_accelerometer_driver(LIS3DSH);

	output_data = init_output_data(OUT_USB);
	output_packet = init_output_packet();

	output_set_send_done_function(output_data, send_done);	//Set the send done function
	output_active(output_data); 							//Enable output
	while(1){
		osDelay(5);		//Es necesario meter algun delay para no bloquear el resto de cosas (sobretodo la entrada de comandos)
		float32_t* data_ptr = (float32_t*) catch_packet_payload_ptr(output_packet);	//Get the data pointer. In this case is a float32, but could ve anything
		uint64_t timestamp = get_timestamp(time_control);							//Get the timestamp when starts capturing from the accelerometer
		accelerometer_driver->accel_funcs.accel_driver_read_stream_samples_poll(accelerometer_driver);
		uint16_t sample_number =accel_get_sample_number(accelerometer_driver);
		for(i=0; i<sample_number*3; i += 3){								//Fill the data pointer. Simulates the accelerometer data. XYZ
			data_ptr[i] = accel_get_x_value(accelerometer_driver, i/3);
			data_ptr[i+1] =  accel_get_y_value(accelerometer_driver, i/3);
			data_ptr[i+2] = accel_get_z_value(accelerometer_driver, i/3);
		}
		release_packet_payload_ptr(output_packet, (uint8_t*) data_ptr);			//Release the data pointer. Now it has been filled

		set_packet_attr(output_packet, timestamp, sample_number*3, sizeof(float32_t), ACCELEROMETER);	//Add data attributes to the packet. Now the packet is ready to be sent
		output_set_data_ready(output_data);								//Set data ready

		output_data->output_data_func(output_data,(uint8_t*) output_packet, output_packet->packet_header.total_num_bytes);//Send the full packet with the header


//		lis3dh_driver->accel_funcs.accel_driver_read_stream_samples_it(lis3dh_driver, DATA_SIZE, xAxis, yAxis, zAxis);
//		usb_printf("%d Samples\r\n", DATA_SIZE);
//		for(i=0; i<DATA_SIZE; i++){
//			usb_printf("X=%.3fg Y=%.3fg Z=%.3fg\r\n", xAxis[i], yAxis[i], zAxis[i]);
//			osDelay(20);
//		}
	}
}


retval_t send_done(struct output_data_* output_data){
	output_reset_data_ready(output_data);					//Reset the data ready
	leds_toggle(LEDS_BLUE);
	return RET_OK;
}
