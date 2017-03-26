/*
 * spi.h
 *
 *  Created on: 25 Mar 2017
 *      Author: dupa
 */

#ifndef SPI_H_
#define SPI_H_


void initSPI();
void startComm();
void endComm();
void sendByteRaw(unsigned char data);
unsigned char sendByte(unsigned char data);
unsigned char readByteRaw();
void readBytes(unsigned char buffer[], unsigned char initByte, unsigned int size);
#endif /* SPI_H_ */
