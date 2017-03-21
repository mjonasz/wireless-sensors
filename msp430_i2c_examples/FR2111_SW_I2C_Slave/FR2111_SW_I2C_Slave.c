/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//  MSP430FR2111 Software I2C Slave
//
//  Description: This demo connects two MSP430's via the I2C bus. The master is
//  MSP430FR2311 with hardware I2C(eUSCI_B0). The slave is MSP430FR2111 using GPIOs
//  to implement software I2C. The master write to and read from the slave.
//  This is the slave code.
//  MCLK = SMCLK = 8MHz.
//
//    *****used with "FR2311_HW_I2C_Master.c"****
//
//                                /|\  /|\
//               MSP430FR2311      10k  10k     MSP430FR2111
//                   master         |    |        slave
//             -----------------   |    |   -----------------
//            |     P1.2/UCB0SDA|<-|----|->|P2.0(SW I2C)     |
//            |                 |  |       |                 |
//            |                 |  |       |                 |
//            |     P1.3/UCB0SCL|<-|------>|P1.0(SW I2C)     |
//            |                 |          |                 |
//
//   Texas Instruments Inc.
//   June. 2016
//******************************************************************************
#include <msp430.h> 

#define SCL BIT0
#define SDA BIT0
#define I2COA 0x0A               // Slave address

#define I2C_START     1          // Start condition
#define I2C_STOP      2          // Stop condition
#define SCL_W1LH      3          // Bit 1 (first clk rising edge)
#define SCL_W1HL      4          // Bit 1 (first clk falling edge)
#define SCL_W2to6LH   5          // Bit 2-6(2nd~6th  clk rising edge)
#define SCL_W7LH      6          // Bit 7 (7th  clk rising edge)
#define SCL_W8LH      7          // Bit 8 (8th  clk rising edge)
#define SCL_W8HL      8          // Set ACK (8th  clk falling edge)
#define SCL_W9HL      9          // Re-init R4 (9th  clk falling edge)
#define SCL_R1HL      10         // Bit 1 (first clk falling edge)
#define SCL_R1LH      11         // Bit 1 (first clk rising edge)
#define SCL_R2to8HL   12         // Bit 2-8 (2nd~8th clk falling edge)
#define SCL_R9HL      13         // Free SDA (9th clk falling edge)
#define SCL_R9LH      14         // Check ACK (9th clk rising edge)

#define MCLK_FREQ_MHZ 8          // MCLK = 8MHz

unsigned char ram_data[16];
unsigned char i2c_data = 0;
unsigned int I2C_state = 0;
unsigned char cnt_2to6=0;
unsigned char cnt_2to8=0;
unsigned int RW_flag=0;
unsigned char ram_cnt=0;

void InitBuffer();
void INIT_PORT();
void Software_Trim();

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	            // Stop watchdog timer

    __bis_SR_register(SCG0);                // Disable FLL
    CSCTL3 = SELREF__REFOCLK;               // Set REFO as FLL reference source
    CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;// DCOFTRIM=3, DCO Range = 8MHz
    CSCTL2 = FLLD_0 + 243;                  // DCODIV = 8MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                // Enable FLL
    Software_Trim();                        // Software Trim to get the best DCOFTRIM value
    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                               // default DCODIV as MCLK and SMCLK source
	InitBuffer();
	INIT_PORT();

	__enable_interrupt();
	while(1);
}

void InitBuffer()
{
	unsigned char i;
	for(i=0;i<16;i++)
	{
		ram_data[i] = 1;
	}
}

// Set SCL and SDA to inputs
// SDA interrupts on high-to-low transition
// SCL interrupts on low-to-high transition
// initially, just SDA interrupt is set
void INIT_PORT()
{
	P2OUT &= ~SDA;    // When the pins are set to
	P1OUT &= ~SCL;    // output, low level should be seen
	P1DIR &= ~SCL;    // SCL and SDA defined as inputs
	P2DIR &= ~SDA;
	P2IES |= SDA;     //INT. on high-to-low transition
	P1IES &= ~SCL;    // INT. on low-to-high transition
	P1IE  &= ~SCL;    // Disable SCL interrupt
	P2IE  |= SDA;     // Enable SDA interrupt
	P1IFG &= ~SCL;    // Reset interrupt flag
	P2IFG &= ~SDA;

	PM5CTL0 &= ~LOCKLPM5;  // Disable the GPIO power-on default high-impedance mode
	                       // to activate previously configured port settings
}

// SDA(Port 2) interrupt service routine
// Port 2 ISR, check Start or Stop condition from P2.0 and P1.0
// Or it can check I/O interrupt from other P2 pins with interrupt function
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
	if((P1IN & SCL == 1) && (P2IFG & SDA == 1) && (P2IES & SDA == 1))
	{
		I2C_state = I2C_START; // I2C start condition
	}
	else
	{
		I2C_state = I2C_STOP;
	}
	P2IFG = 0;          // Clear INT flag

	switch(I2C_state)
	{
	case I2C_START:
		P1IFG &= ~SCL;  // P1.0 clear INT flag
		P1IE |= SCL;    // P1.0 INT enable for SCL
		P2IES &= ~SDA;  // P2.0 set to rising edge for SDA
		P2IFG &= ~SDA;  // P2.0 clear INT flag
		P2IE &= ~ SDA;  // P2.0 INT disable for SDA
		I2C_state = SCL_W1LH;
		break;
	case I2C_STOP:
		INIT_PORT();
		i2c_data = 0;   // Reset I2CDATA for addr detect
		RW_flag = 0;    // Reset RW_flag for addr detect
		ram_cnt = 0;    // Reset buffer pointer
		break;
	default:
		break;
	}
}

// SCL(Port 1) interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
	switch(I2C_state)
	{
	case SCL_W1LH:
		P1IES |= SCL;      // Set falling edge for SCL
		if(P2IN & SDA)     // Check SDA and update i2c_data
			i2c_data = 1;  // Shift SDA bit into the buffer
		else
			i2c_data = 0;
		P1IFG &= ~SCL;     // Reset SCL interrupt flag
		P2IFG &= ~SDA;     // Clear flag, FOR STP_CON DETECT
		P2IE |= SDA;       // Enable SDA INT
		I2C_state = SCL_W1HL;
		break;
	case SCL_W1HL:
		P1IES &= ~ SCL;    // Set rising edge for SCL
		P1IFG &= ~SCL;     // Reset SCL interrupt flag
		P2IE &= ~SDA;      // 1st FALLING SCL TO disable SDA INT
		I2C_state = SCL_W2to6LH;
		break;
	case SCL_W2to6LH:
		P1IFG &= ~SCL;     // Reset interrupt flag
		i2c_data <<= 1;
		if(P2IN & SDA)     // Check SDA and and update i2c_data
			i2c_data++;
		if(cnt_2to6 < 4)   // 5bits(bit2 to bit6)
			cnt_2to6++;
		else
		{
			cnt_2to6 = 0;  // Reset counter
			I2C_state = SCL_W7LH;
		}
		break;
	case SCL_W7LH:
		P1IFG &= ~SCL;     // Reset interrupt flag
		i2c_data <<= 1;
		if(P2IN & SDA)     // Check SDA and and update i2c_data
		{
			i2c_data++;
		}
		if((RW_flag == 0) && (i2c_data != I2COA) )
			I2C_state = I2C_STOP;
		else
			I2C_state = SCL_W8LH;
		break;
	case SCL_W8LH:
		P1IES |= SCL;        // Set falling edge for SCL.
		P1IFG &= ~SCL;       // Reset interrupt flag
		if(RW_flag == 0)
		{
			if(P2IN & SDA)   // Check SDA and and update i2c_data
				RW_flag = 1; // Read CMD
			else
				RW_flag = 2; // Write CMD
		}
		else
		{
			i2c_data <<= 1;
			if(P2IN & SDA)   // Check SDA and and update i2c_data
				i2c_data++;
		}
		I2C_state = SCL_W8HL;
		break;
	case SCL_W8HL:
		P2DIR |= SDA;        // Output "0" for SDA
		P1IFG &= ~SCL;       //Reset SCL interrupt flag
		if(RW_flag == 1)
		{
			i2c_data = ram_data[ram_cnt];
			ram_cnt++;
            ram_cnt &= 0x0f;
			I2C_state = SCL_R1HL;// Go to read state
		}
		else if(RW_flag == 2)
		{
			RW_flag = 3;
			I2C_state = SCL_W9HL;// Go to write state
		}
		else if(RW_flag == 3)
		{
			ram_data[ram_cnt] = i2c_data;
			ram_cnt++;
            ram_cnt &= 0x0f;
			I2C_state = SCL_W9HL;// Go to write state
		}
		else
			I2C_state = I2C_STOP;
		break;
	case SCL_W9HL:
		P2DIR &= ~SDA;        // Release SDA
		P1IES &= ~SCL;        // Set rising edge for SCL
		P1IFG &= ~SCL;        // Reset interrupt flag
		I2C_state = SCL_W1LH;
		break;
	case SCL_R1HL:
		P1IES &= ~SCL;        // Set rising edge SCL for SCL_R1LH
		if(i2c_data & BIT7)
			P2DIR &= ~SDA;    // SDA = 1
		else
			P2DIR |= SDA;     // SDA = 0
		i2c_data <<= 1;
		P1IFG &= ~ SCL;       // Clear SCL INT flag
		P2IE  &= ~ SDA;       // Disable SDA INT, NO STP CON
		I2C_state = SCL_R1LH;
		break;
	case SCL_R1LH:
		P1IES |= SCL;         // Set falling edge for SCL
		P1IFG &= ~SCL;        // Clear SCL INT flag
		P2IFG &= ~SDA;        // Clear SDA INT Flag
		P2IE |= SDA;          // Enable SDA INT for STOP CON
		I2C_state = SCL_R2to8HL;
		break;
	case SCL_R2to8HL:
		if(i2c_data & BIT7)
			P2DIR &= ~SDA;    // SDA = 1
		else
			P2DIR |= SDA;     // SDA = 0
		i2c_data <<= 1;
		P1IFG &= ~ SCL;       // Clear SCL INT flag
		P2IE  &= ~ SDA;       // Disable SDA INT, NO STP CON
		if(cnt_2to8 < 6)
		{
			cnt_2to8++;       // 7bits(bit2 to bit8)
		}
		else
		{
			cnt_2to8 = 0;     // Clear counter
			I2C_state = SCL_R9HL;
		}
		break;
	case SCL_R9HL:
		P2DIR &= ~SDA;        // Release SDA
		P1IES &= ~SCL;        // Set Rising edge of SCL
		P1IFG &= ~SCL;        // Clear SCL INT flag
		I2C_state = SCL_R9LH;
		break;
	case SCL_R9LH:
		if(P2IN & SDA)        // NACK_READ
		{
			P1IES &= ~SCL;    // Set rising edge of SCL
			P1IFG &= ~SCL;    // Clear SCL INT flag
			I2C_state = SCL_W1LH; // Go to SCL_W1LH to detect stop condition
		}
		else                  // ACK_READ
		{
			P1IES |= SCL;     // Set falling edge of SCL
			P1IFG &= ~SCL;    // Clear SCL INT flag
			i2c_data = ram_data[ram_cnt]; // Move out next byte from RAM buffer
			ram_cnt++;
			ram_cnt &= 0x0f;
			I2C_state = SCL_R1HL;// Go to read state
		}
		break;
	case I2C_STOP:
		INIT_PORT();
		i2c_data = 0;   // Reset I2CDATA for addr detect
		RW_flag = 0;    // Reset RW_flag for addr detect
		ram_cnt = 0;    // Reset buffer pointer
		break;
	default:
		break;
	}
}




// Software Trim to get the best DCOFTRIM value for FLL lock
void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);// Wait FLL lock status (FLLUNLOCK) to be stable
                                                           // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;       // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}

