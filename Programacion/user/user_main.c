/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include "user_config.h"
#include "i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_common.h"
#include "stdint.h"


/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    printf("SDK version:%s\n", system_get_sdk_version());
}

void Control_TASK(void){
	//Formato Q15 (180º)
	//Accion = (Kp + Ki/(1-z^-1))Error + Kd(1-z^-1)Pos
	const uint32_t Kp[4] = {2200, 2, 2, 4}; //Ajustar
	const uint32_t Kv[4] = {2, 2, 2, 4}; //Ajustar
	const uint32_t Ki[4] = {2, 2, 2, 4}; //Ajustar	
	
	int16_t Accion[4] = { 0, 0, 0, 0};
	uint16_t AccionMotor[4] = { 0, 0, 0, 0};
	
	int16_t Referencia[4] = { 0, 0, 0, 0};
	int16_t Posicion[4][3] = { 0, 0, 0, 0};
	int16_t Error[4][2] = { 0, 0};
	
	while(1){
		printf("SDK version:%s\n", system_get_sdk_version());
		
			//PI-D Eje 0
		Posicion[0][2] = Posicion[0][1];	
		Posicion[0][1] = Posicion[0][0];
		//Posicion[0][0] =  medir	
		
		Error[0][1] = Error[0][0];
		Error[0][0] = Referencia[0] - Posicion[0][0];
		 	 
		Accion[0] += (Kp[0] * (Error[0][0] - Error[0][1]));
		Accion[0] += (Kv[0] * (Posicion[0][0] - ((int32_t)Posicion[0][1] << 1) + Posicion[0][2]));		
		Accion[0] += (Ki[0] * Error[0][0]);
		
				//PI-D Eje 1
		Posicion[1][2] = Posicion[1][1];
		Posicion[1][1] = Posicion[1][0];
		//Posicion[1][0] =  medir	
		
		Error[1][1] = Error[1][0];
		Error[1][0] = Referencia[1] - Posicion[1][0];
		 	 
		Accion[1] += (Kp[1] * (Error[1][0] - Error[1][1]));
		Accion[1] += (Kv[1] * (Posicion[1][0] - ((int32_t)Posicion[1][1] << 1) + Posicion[1][2]));		
		Accion[1] += (Ki[1] * Error[1][0]);
		
				//PI-D Eje 2
		Posicion[2][2] = Posicion[2][1];
		Posicion[2][1] = Posicion[2][0];
		//Posicion[2][0] =  medir	
		
		Error[2][1] = Error[2][0];
		Error[2][0] = Referencia[2] - Posicion[2][0];
		 	 
		Accion[2] += (Kp[2] * (Error[2][0] - Error[2][1]));
		Accion[2] += (Kv[2] * (Posicion[2][0] - ((int32_t)Posicion[2][1] << 1) + Posicion[2][2]));		
		Accion[2] += (Ki[2] * Error[1][0]);
		
				//PI-D Eje 3
		Posicion[3][2] = Posicion[3][1];
		Posicion[3][1] = Posicion[3][0];
		//Posicion[3][0] =  medir	
		
		Error[3][1] = Error[3][0];
		Error[3][0] = Referencia[3] - Posicion[3][0];
		 	 
		Accion[3] += (Kp[3] * (Error[3][0] - Error[3][1]));
		Accion[3] += (Kv[3] * (Posicion[3][0] - ((int32_t)Posicion[3][1] << 1) + Posicion[3][2]));		
		Accion[3] += (Ki[3] * Error[1][0]);
		
		AccionMotor[0] =  Accion[0] >> 2 + Accion[1] >> 2 + Accion[2] >> 2 + Accion[3] >> 2;
		AccionMotor[1] =  Accion[0] >> 2 - Accion[1] >> 2 - Accion[2] >> 2 + Accion[3] >> 2;
		AccionMotor[2] = -Accion[0] >> 2 - Accion[1] >> 2 + Accion[2] >> 2 + Accion[3] >> 2;
		AccionMotor[3] = -Accion[0] >> 2 + Accion[1] >> 2 - Accion[2] >> 2 + Accion[3] >> 2;
	}
}