# Wireless sensors

## Reading BME280 (temperature, humidity, pressure)

Reading will be done using spi protocol. Sensing interval should be about 1/min.

* BST-BME280_DS001-10.pdf

## Wireless communication

Communication will be done using 433MHz raw transmitter with Manchester encoding at MCU level. 

* http://electronics.stackexchange.com/questions/173170/using-433-mhz-tx-rx-without-encoders-decoders
* Preamble before encoding: 0x02
* Preamble after encoding: 0b10101010 0b10011001 (may be optimized)
* Effective speed ~2000bps

## Experiments

* When tested with rtl_433 I discovered that there are couple near meteo stations in my area, so we need to be aware that our 433mhz transmitter is not the only one in neightborhood and we have to deal with some interferences (simplest solution is to send our data couple times in some random intervals).
* Tested receiving using MX-05V powered from laptop USB. It needs strong filtering. It works very stable with average voltage 5.14v and 0.04v amplitude (laptop usb). Filtering was created from capacitors and ferride bead. This kind of filtering wasn't enough to get good results from standard usb charger. MX-05V is only for temporary use, later that will be SRX882.
* STX882 (~1500bps) trasmitter is slower at 3.5v than FS1000A (~2000bps), so FS1000A looks like better, simpler and cheaper solution
* Tests with rtl_433 and Manchester encoding show that even without checksum there are no wrong data received. If similar receiving implementation will be created for esp01, then CRC8 will be enough to make sure that no wrong data will be ever received. We can drop whole packet if there is any inconformity with Manchaster encoding (rtl_433 works that way).

## Done

* Sending test message with Manchester encoding (works very reliable when receiving with rtl_433)
* Reading BME280 sensor (spi@msp430)
* Basic implementation for transmitting readings from sensor
* Basic script for receiving (rtl_433) and printing human readable sensor readings (it works)

## Variations

### Premium 

* mcu: MSP430G2452 (1.5$)
* sensor temperature, humidity, pressure: BME280 (3.2$)
* transmitter: FS1000A (0.5$)
* battery: CR2032 (0.2$)

### Cheap 

* mcu: MSP430G2231 (0.57$), i2c, spi, 2KB rom, 128B ram
* sensor temperature only: TMP112 (0.55$), 0.5°C precision
* transmitter: FS1000A (0.5$)
* battery: CR2032 (0.2$)

## TODO

* Develop transmission format
* Implement sending sensor data (every reading should be sent couple times in random intervals) - test version done
* Implement receiving and parsing of sensor readings (in rtl_433) - test version done
* Implement sleep modes for intervals between reading/sending sensor data
* Implement reading device ID with some physical jumpers or whatever (then use ID as seed to PRNG for generating intervals between transmitted data)
* Implement receiving of sensor readings with some IC (probably esp01) and 433mhz receiver (SRX882 should work good at 3.3v)
* Some simple HMAC mechanism to prevent tempering with reading? At once it will check for any transmission errors.
* Graphs with InfluxDB and Grafana 
