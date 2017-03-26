/*
 * bme280.c
 *
 *  Created on: 25 Mar 2017
 *      Author: dupa
 */

#include <bme280.h>
#include <spi.h>

//CONFIG
#define CTRL_HUM_ADR 0xF2
#define CTRL_HUM_VAL 0b001 // hum x1 001

#define CTRL_MEAS_ADR 0xF4
#define CTRL_MEAS_VAL 0b00100101 // temp x1 001, press x1 001, forced mode 01

//STATUS
#define STATUS_ADR 0xF3
#define STATUS_IN_PROGRESS_BIT (1<<3)

//READ
#define PRESS_MSB_ADR 0xF7
#define PRESS_LSB_ADDR 0xF8
#define PRESS_XLSB_ADDR 0xF9 // 4 MSB only

#define TEMP_MSB_ADR 0xFA
#define TEMP_LSB_ADR 0xFB
#define TEMP_XLSB_ADR 0xFC // 4 MSB only

#define HUM_MSB_ADR 0xFD
#define HUM_LSB_ADR 0xFE

//calibration data
#define CALIB1_START_ADR 0x88
#define CALIB1_END_ADR 0x0A1
#define CALIB1_SIZE (CALIB1_END_ADR-CALIB1_START_ADR+1)
#define CALIB2_START_ADR 0xE1
#define CALIB2_END_ADR 0xF0
#define CALIB2_SIZE (CALIB2_END_ADR-CALIB2_START_ADR+1)

#define REG_WRITE_MASK_AND 0b01111111
#define REG_READ_MASK_OR 0b10000000

// measure max time 1.25+2.3 + 2.3+0.575 + 2.3+0.575 = 9.3ms |== ~10ms

void measure(unsigned char buffer[]){
    //CSB UP
    //setup HUM register
    startComm();
    sendByteRaw(CTRL_HUM_ADR & REG_WRITE_MASK_AND);
    sendByteRaw(CTRL_HUM_VAL);
    //setup MEAS register
    sendByteRaw(CTRL_MEAS_ADR & REG_WRITE_MASK_AND);
    sendByteRaw(CTRL_MEAS_VAL);
    endComm();
    // wait 10ms
    __delay_cycles(10000);
    //optional check if data ready (STATUS)
    //read data
    readBytes(buffer, PRESS_MSB_ADR | REG_READ_MASK_OR, 8);
}

void readCalibrationData(){
    unsigned char calib1[CALIB1_SIZE];
    unsigned char calib2[CALIB2_SIZE];
    readBytes(calib1, CALIB1_START_ADR | REG_READ_MASK_OR, CALIB1_SIZE);
    readBytes(calib2, CALIB2_START_ADR | REG_READ_MASK_OR, CALIB2_SIZE);
}
