/*
 * accelerometer_config.c
 *
 *  Created on: 8 de feb. de 2017
 *      Author: jblesa
 */

#include "yetimote-conf.h"
#include "commands.h"
#include "accelerometer_driver.h"

extern accelerometer_driver_t* accelerometer_driver;

retval_t acc_config(uint16_t argc, char** argv) {

	if(argc == 1){
		UsbWriteString("Command error. Missing params\r\n");
		UsbWriteString("Write -h for command usage\r\n");
	}
	///Con solo el nombre y otro argumento la única opción es haber marcado -h. En caso contrario error
	else if(argc == 2){
		if (strcmp(argv[1], "-h") == 0){
			UsbWriteString("Usage: ");
			UsbWriteString(ACC);
			UsbWriteString(" [set/get]\r\n");
			UsbWriteString("Configure accelerometer parameters\r\n\r\n");
			UsbWriteString("\t-h\t: print command help\r\n");
			UsbWriteString("\t  \t: odr\r\n");
			UsbWriteString("\t  \t: mode\r\n");
			UsbWriteString("\t  \t: fs\r\n");
			UsbWriteString("\t  \t: activeacc\r\n");
			UsbWriteString("Usage Example:\r\n");
			UsbWriteString("\t");
			UsbWriteString(ACC);
			UsbWriteString(" set odr 100\r\n");
			UsbWriteString("\tThis will change the odr to 100Hz\r\n\r\n");
		}
	}
	else if (argc == 3){
		if (strcmp(argv[2], "-h") == 0){
			//XXX-jblesa Aquí la ayuda de cada comando
		}
		else if (strcmp(argv[1], "get")==0){
			//XXX-jblesa los comandos get solo tienen 3 argumentos
			if (strcmp(argv[2], "odr")==0){

			}
			else if(strcmp(argv[2], "mode")==0){

			}
			else if(strcmp(argv[2], "fs")==0){

			}
			else if(strcmp(argv[2], "activeacc")==0){

			}
			else {
				UsbWriteString("Command error. Wrong params\r\n");
				UsbWriteString("Write -h for command usage\r\n");
			}
		}

	}
	else if (argc == 4) {
		if (strcmp(argv[1], "set")==0){
			if (strcmp(argv[2], "odr")==0){
				uint16_t new_odr = (uint16_t) atoi(argv[3]);
				accelerometer_driver->accel_funcs.accel_driver_set_odr(accelerometer_driver, new_odr);
			}
			else if(strcmp(argv[2], "mode")==0){

			}
			else if(strcmp(argv[2], "fs")==0){

			}
			else if(strcmp(argv[2], "activeacc")==0){

				accelerometer_type_t accel_type = accel_get_type(accelerometer_driver);

				if (strcmp(argv[3], "lis3dsh")==0){
					if(accel_type != LIS3DSH){
						delete_accelerometer_driver(accelerometer_driver);
						accelerometer_driver = new_accelerometer_driver(LIS3DSH);
					}

				}
				else if (strcmp(argv[3], "lis3dh")==0){
					if(accel_type != LIS3DH){
						delete_accelerometer_driver(accelerometer_driver);
						accelerometer_driver = new_accelerometer_driver(LIS3DH);
					}
				}
				else if (strcmp(argv[3], "mma8652")==0){
//					if(accel_type != MMA8652){	//DRIVER NOT IMPLEMENTED YET
//
//					}
				}
				else if (strcmp(argv[3], "adxl355")==0){
//					if(accel_type != ADXL355){	//DRIVER NOT IMPLEMENTED YET
//
//					}
				}
				else{
					UsbWriteString("Command error. Wrong params\r\n");
					UsbWriteString("Write -h for command usage\r\n");
				}
			}
			else if(strcmp(argv[2], "sample_num")==0){
				uint16_t new_sample_num = (uint16_t) atoi(argv[3]);
				accel_set_sample_number(accelerometer_driver, new_sample_num);
			}
			else {
				UsbWriteString("Command error. Wrong params\r\n");
				UsbWriteString("Write -h for command usage\r\n");
			}

		}

		else {
			UsbWriteString("Command error. Wrong params\r\n");
			UsbWriteString("Write -h for command usage\r\n");
		}
	}
	else {
		UsbWriteString("Command error. Wrong params\r\n");
		UsbWriteString("Write -h for command usage\r\n");
	}
	return RET_OK;
}
