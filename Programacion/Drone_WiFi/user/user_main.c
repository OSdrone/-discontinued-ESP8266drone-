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
#include "inv_mpu.h" 
#include "inv_mpu_dmp_motion_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"


#include "esp_common.h"
#include "stdint.h"
#include "math.h"


/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/

void Control_TASK_FCN();
void CLOCK_Control_FCN();

//..........Parametros..............//
static inline unsigned short inv_row_2_scale(const signed char *row){
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx){
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};
										   
//..........Perifericos...........//
//..PWM..//
#define PWM_period 1000
#define PWM_0_OUT_IO_MUX PERIPHS_IO_MUX_GPIO5_U
#define PWM_0_OUT_IO_NUM 5
#define PWM_0_OUT_IO_FUNC  FUNC_GPIO5

#define PWM_1_OUT_IO_MUX PERIPHS_IO_MUX_GPIO4_U
#define PWM_1_OUT_IO_NUM 4
#define PWM_1_OUT_IO_FUNC  FUNC_GPIO4

#define PWM_2_OUT_IO_MUX PERIPHS_IO_MUX_MTDO_U
#define PWM_2_OUT_IO_NUM 15
#define PWM_2_OUT_IO_FUNC  FUNC_GPIO15

#define PWM_3_OUT_IO_MUX PERIPHS_IO_MUX_MTMS_U
#define PWM_3_OUT_IO_NUM 14
#define PWM_3_OUT_IO_FUNC  FUNC_GPIO14

static uint32 PWM_Dutty[4] = {0, 0, 0, 0};

uint32 io_info[][3] = {   {PWM_0_OUT_IO_MUX,PWM_0_OUT_IO_FUNC,PWM_0_OUT_IO_NUM},
                          {PWM_1_OUT_IO_MUX,PWM_1_OUT_IO_FUNC,PWM_1_OUT_IO_NUM},
                          {PWM_2_OUT_IO_MUX,PWM_2_OUT_IO_FUNC,PWM_2_OUT_IO_NUM},
						  {PWM_3_OUT_IO_MUX,PWM_3_OUT_IO_FUNC,PWM_3_OUT_IO_NUM}
                          };


//..........Tareas................//
//..Control..//
xSemaphoreHandle SEMAPHORE_Control = NULL;
xTaskHandle TASK_Control = NULL;
xTimerHandle CLOCK_Control = NULL;

#define Task_control_stack 256 
#define Task_control_priority 9 
#define Task_control_period 10 
#define Task_control_start_time 100



void user_init(void)
{
    printf("SDK version:%s\n", system_get_sdk_version());
	
	//Tarea_Control
	vSemaphoreCreateBinary(SEMAPHORE_Control);
	xTaskCreate(Control_TASK_FCN, "TAREA CONTROL", 1, NULL, Task_control_priority, &TASK_Control);
	CLOCK_Control = xTimerCreate("TIMER_CLOCK", Task_control_period, pdTRUE, (void *)0, CLOCK_Control_FCN);
	xTimerStart(CLOCK_Control, Task_control_start_time); //Arranque del timer
	
	//Arranque PWM
	pwm_init(PWM_period, PWM_Dutty, 4, io_info); //REVISAR SI SE HA DE ACTIVAR INDIVIDUALMENTE
	pwm_start();
	
	//Arranque MPU
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(100);
	dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
	dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL);
	dmp_set_fifo_rate(100);
    mpu_set_dmp_state(1);
	
	//Arranque Sistema
	vTaskStartScheduler(); 
}

void Control_TASK_FCN(void){
    unsigned char more;
    long quat[4];
	
	unsigned long sensor_timestamp;
			
	//Formato 2000? -> 2^15
	//Accion = (Kp + Ki/(1-z^-1))Error + Kd(1-z^-1)Pos
	
	const uint16_t Kp[4] = {2200, 2, 2, 4}; //Ajustar
	const uint16_t Kv[4] = {2, 2, 2, 4}; //Ajustar
	const uint16_t Ki[4] = {2, 2, 2, 4}; //Ajustar	
	
	int16_t Accion[4] = { 0, 0, 0, 0};
	//uint16_t AccionMotor[4] = { 0, 0, 0, 0};
	
	int16_t Referencia[4] = { 0, 0, 0, 0};
	int16_t Posicion[4][3] = { 0, 0, 0, 0};
	int16_t Error[4][2] = {0, 0};
	
	while(1){
		xSemaphoreTake(SEMAPHORE_Control, portMAX_DELAY);
		
		/* Leer Ref */
		Posicion[0][2] = Posicion[0][1];	
		Posicion[0][1] = Posicion[0][0];
		Posicion[1][2] = Posicion[1][1];
		Posicion[1][1] = Posicion[1][0];
		Posicion[2][2] = Posicion[2][1];
		Posicion[2][1] = Posicion[2][0];	
		Posicion[3][2] = Posicion[3][1];
		Posicion[3][1] = Posicion[3][0];
		
		//Leer posicion
		dmp_read_fifo(NULL, NULL, quat, &sensor_timestamp, NULL, &more);
		
		Posicion[0][0] =/* M_PI/180.0 * 32768.0*/ * atan2( (((quat[0]>>15)*(quat[1]>>15) + (quat[2]>>15)*(quat[3]>>15))>>14)/32768.0 , 1.0 - (((quat[1]>>15)*(quat[1]>>15) + (quat[2]>>15)*(quat[2]>>15))>>14) );
		Posicion[1][0] =/* M_PI/180.0 * 32768.0*/ * asin( ( ((quat[0]>>15)*(quat[2]>>15) - (quat[3]>>15)*(quat[1]>>15)) >>14) /32768.0 );
		Posicion[2][0] =/* M_PI/180.0 * 32768.0*/ * atan2( (((quat[0]>>15)*(quat[3]>>15) + (quat[1]>>15)*(quat[2]>>15))>>14)/32768.0 , 1.0 - (((quat[2]>>15)*(quat[2]>>15) + (quat[3]>>15)*(quat[3]>>15))>>14) );
		
		//Leer Altura
		
		//PI-D Eje 0	
		
		Error[0][1] = Error[0][0];
		Error[0][0] = Referencia[0] - Posicion[0][0];
		 	 
		Accion[0] += (Kp[0] * (Error[0][0] - Error[0][1]));
		Accion[0] += (Kv[0] * (Posicion[0][0] - ((int32_t)Posicion[0][1] << 1) + Posicion[0][2]));		
		Accion[0] += (Ki[0] * Error[0][0]);
		
		//PI-D Eje 1

		Error[1][1] = Error[1][0];
		Error[1][0] = Referencia[1] - Posicion[1][0];
		 	 
		Accion[1] += (Kp[1] * (Error[1][0] - Error[1][1]));
		Accion[1] += (Kv[1] * (Posicion[1][0] - ((int32_t)Posicion[1][1] << 1) + Posicion[1][2]));		
		Accion[1] += (Ki[1] * Error[1][0]);
		
		//PI-D Eje 2

		Error[2][1] = Error[2][0];
		Error[2][0] = Referencia[2] - Posicion[2][0];
		 	 
		Accion[2] += (Kp[2] * (Error[2][0] - Error[2][1]));
		Accion[2] += (Kv[2] * (Posicion[2][0] - ((int32_t)Posicion[2][1] << 1) + Posicion[2][2]));		
		Accion[2] += (Ki[2] * Error[1][0]);
		
		//PI-D Eje 3
	
		Error[3][1] = Error[3][0];
		Error[3][0] = Referencia[3] - Posicion[3][0];
		 	 
		Accion[3] += (Kp[3] * (Error[3][0] - Error[3][1]));
		Accion[3] += (Kv[3] * (Posicion[3][0] - ((int32_t)Posicion[3][1] << 1) + Posicion[3][2]));		
		Accion[3] += (Ki[3] * Error[1][0]);
		
		PWM_Dutty[0] =  Accion[0] >> 2 + Accion[1] >> 2 + Accion[2] >> 2 + Accion[3] >> 2;
		PWM_Dutty[1] =  Accion[0] >> 2 - Accion[1] >> 2 - Accion[2] >> 2 + Accion[3] >> 2;
		PWM_Dutty[2] = -Accion[0] >> 2 - Accion[1] >> 2 + Accion[2] >> 2 + Accion[3] >> 2;
		PWM_Dutty[3] = -Accion[0] >> 2 + Accion[1] >> 2 - Accion[2] >> 2 + Accion[3] >> 2;
		
		pwm_set_duty(PWM_Dutty[0], 0);
		pwm_set_duty(PWM_Dutty[1], 1);
		pwm_set_duty(PWM_Dutty[2], 2);
		pwm_set_duty(PWM_Dutty[3], 3);
	}
}
void CLOCK_Control_FCN(void){
	xSemaphoreGive(SEMAPHORE_Control);
}
