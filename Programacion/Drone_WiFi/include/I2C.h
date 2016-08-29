/**
  ******************************************************************************
  * @file    I2C.h
  * @author  OSDrone
  * @version 1
  * @date    Ago 28, 2016
  *
  ******************************************************************************

 */
 
#include "stdint.h"
#include "stdbool.h"

#ifndef I2C_H_
#define I2C_H_
 
/**
 * @brief   Initialize I2C GPIO pins
*/
void I2C_init(void);

/**
 * @brief   Send I2C start bit
 * @return  true if start successfully. Otherwise the bus is busy
 */
bool I2C_start(void);

/**
 * @brief   Send I2C stop bit
 */
void I2C_stop(void);

/**
 * @brief    Send data to I2C bus
 * @param    data Data to send
 * @return   true if ACK is received at end of sending. False if not ACK'ed
 */
bool I2C_write_byte(uint8_t data);

/**
 * @brief    Read data from I2C bus
 * @return   Data read
 */
uint8_t I2C_read_byte(void);

/**
 * @brief   Send acknowledge bit (at end of reading)
 * @param   ack ACK (true) or NACK (false)
 */
void I2C_set_ack(bool ack);

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
#endif