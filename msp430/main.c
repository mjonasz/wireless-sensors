#include <msp430.h>
#include <spi.h>
#include <bme280.h>

// FREQ = 1MHZ/DELAY_CYCLES(200) = 5000bps/2
//#define DELAY_CYCLES 300  // STX882 at 3.5v
#define DELAY_CYCLES 200 // FS1000A at 3.5v
#define CODE 0b10101010
#define RF_TX_PIN 0
#define uint unsigned int
#define uchar unsigned char

void initConfiguration()
{
    DCOCTL = CALDCO_1MHZ;
    BCSCTL1 = CALBC1_1MHZ;

    WDTCTL = WDTPW | WDTHOLD;       // Stop watchdog timer
    P1DIR |= 1 << RF_TX_PIN;

    initSPI();
}

void txDelay()
{
    __delay_cycles(DELAY_CYCLES);
}

void sendChar(uchar data)
{
    int i;
    for (i = 7; i >= 0; i--)
    {
        P1OUT ^= (-((data >> i) & 1) ^ P1OUT) & (1 << RF_TX_PIN);
        txDelay();
        P1OUT ^= 1 << RF_TX_PIN;
        txDelay();
    }
}

void sendData(const uchar data[], int len)
{

    int i;
//    for (i = len - 1; i >= 0; i--){
//        data[i] ^= CODE;
//    }

    sendChar(0b11111010); //preamble

    for (i = len; i > 0; i--)
        sendChar(*(data++));

    P1OUT &= ~(1 << RF_TX_PIN);
}

void sendMeasurement(uchar id, uchar data[], const int len)
{
    //TODO add ID
    sendData(data, 8);
}

int main(void)
{
    initConfiguration();
    uchar measurementData[8];
    for (;;)
    {
        measure(measurementData);
        sendData(measurementData, 8);
//        sendMeasurement(1, measurementData, 8);
//        readCalibrationData();
        __delay_cycles(1000000);
    }

    return 0;
}
