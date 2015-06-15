/* 
 * File:   masterI2C.h
 * Author: Harpal
 *
 * Created on 22 gennaio 2015, 21.58
 */

#ifndef MASTERI2C_H
#define	MASTERI2C_H

void i2c_init(void);

void i2c_idle(void);
void i2c_start(unsigned char stype);

void i2c_stop(void);
unsigned char i2c_slave_ack(void);

void i2c_write(unsigned char data);
void i2c_master_ack(unsigned char ack_type);

unsigned char i2c_read(void);

void sendByte(unsigned char data, unsigned char address);
unsigned char readByte(unsigned char address);
#endif	/* MASTERI2C_H */

