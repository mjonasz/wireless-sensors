# Wireless sensors

## Reading BMP280/BME280

Reading will be done using i2c protocol. Sensing interval should be about 1/min.

* BST-BME280_DS001-10.pdf
* BST-BMP280-DS001-11.pdf
* msp430_i2c.pdf

## Communication

Communication will be done using 433MHz raw transmitter with Manchester encoding at MCU level. 

* http://electronics.stackexchange.com/questions/173170/using-433-mhz-tx-rx-without-encoders-decoders
* Preamble before encoding: 0x02
* Preamble after encoding: 0b10101010 0b10011001 (may be optimized)
* Effective speed ~2000bps

## Thoughts

* When tested with rtl_433 I discovered that there are couple near meteo stations in my area, so we need to be aware that our 433mhz transmitter is not the only one in neightborhood and we have to deal with some interferences (simplest solution is to send our data couple times in some random intervals).

## Done

* Sending test message with Manchester encoding (works very reliable when receiving with rtl_433)

## TODO

* Reading BME280/BMP280 sensor (i2c)
* Develop transmission format
* Implement sending sensor data (every reading should be sent couple times in random intervals)
* Implement receiving and parsing of sensor readings (in rtl_433)
* Implement sleep modes for intervals between reading/sending sensor data
* Implement reading device ID with some physical jumpers or whatever (then use ID as seed to PRNG)
* Implement receiving of sensor readings with some IC (probably esp01) and 433mhz receiver (needs stable 5v voltage)
* Some simple HMAC mechanism to prevent tempering with reading? At once it will check for any transmission errors.
