/*
 * spi.c
 *
 *  Created on: 16 Mar 2017
 *      Author: dupa
 */
#include <msp430.h>
#include <spi.h>
#include <intrinsics.h>
#include <stdint.h>
#define CSB BIT4

void initSPI()
{
    // need to pull down CS when transmitting
    P1DIR |= CSB;
    endComm();

    // Pins to enable:
    //  P1.6 -- SDO/MOSI Master-out Slave-In
    //  P1.5 -- SCLK serial clock
    // Flags:
    //  most significant bit sent first
    //  enable output
    //  disable USI (for now)
    USICTL0 = USIPE7 | USIPE6 | USIPE5 | USIMST | USIOE | USISWRST;

    // enable interrupts, but set the interrupt flag for now USICKPH CPHA = 0, CPOL default 0
    USICTL1 = USIIE | USIIFG | USICKPH;

    // Clock speed:
    //  Use clock divider 2^4=16.
    //  Use clock source 2 (submain clock?).
    // Polarity and phase settings/flags for the clock:
    //  SPI Mode 0 --- CPOL=0,CPHA=0  --- USICKPH
    //  SPI Mode 1 --- CPOL=0,CPHA=1  --- 0
    //  SPI Mode 2 --- CPOL=1,CPHA=0  --- USICKPL|USICKPH
    //  SPI Mode 3 --- CPOL=1,CPHA=1  --- USICKPL *** this one for DOGS panel
    USICKCTL = USIDIV_0 | USISSEL_2; // div0 = 1MHZ

    // enable USI
    USICTL0 &= ~USISWRST;

    USICNT = 1;    // clear 1st transmit due to errata USI5
    __delay_cycles(50);          // finish clearing (minimum (n+1)*16 cycles)

    // Clear the USI interrupt flag
    USICTL1 &= ~USIIFG;

    // Pause so everything has time to start up properly.
    __delay_cycles(5500);
    __enable_interrupt();
}

void startComm()
{
    P1OUT &= ~CSB;
}

void endComm()
{
    P1OUT |= CSB;
}

void sendByteRaw(unsigned char data)
{
    USISRL = data;
    USICNT = 8;
    __low_power_mode_0();
}

unsigned char sendByte(unsigned char data)
{
    startComm();
    sendByteRaw(data);
    endComm();
    return USISRL;
}

unsigned char readByteRaw() // do not use inline, it ommit return statement
{
    USISR = 0;
    USICNT = 8;
    __low_power_mode_0();
   return USISRL; // nie chciało działać bo brakowało USIPE7 w inicie
}

void readBytes(unsigned char buffer[], unsigned char initByte,
               unsigned int size)
{
    unsigned int i;

    startComm();
    sendByteRaw(initByte);

    for (i = size; i > 0; i--){ // TODO optimization read 16bits at once: USICNT = USI16B + 16;
        *(buffer++) = readByteRaw();
    }

    endComm();
}

/* This interrupt triggers when the USI serial register gets
 * empty.  We use it to wake up the main program. */
#pragma vector = USI_VECTOR
__interrupt void USI_ISR(void)
{
    USICTL1 &= ~USIIFG;         // clear the interrupt flag
    __low_power_mode_off_on_exit(); //wake on exit
}
