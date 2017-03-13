# Wireless sensors

## Reading BMP280/BME280

Reading will be done using i2c protocol.

* BST-BME280_DS001-10.pdf
* BST-BMP280-DS001-11.pdf
* msp430_i2c.pdf

## Communication

Communication will be done using 433MHz raw transmitter with Manchester encoding at MCU level. 

* http://electronics.stackexchange.com/questions/173170/using-433-mhz-tx-rx-without-encoders-decoders
* Preamble before encoding: 0x02
* Preamble after encoding: 0b10101010 0b10011001 (may be optimized)
* Effective speed ~2000bps
