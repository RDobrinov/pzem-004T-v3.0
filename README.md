# PZEM-004T board version 3.0
Arduino interface library for Peacefair PZEM-004T-100A v3.0 ( and maybe PZEM-004T-10A ) AC Communication Box

***

The Power supply from microcontroler power only the communication interface optocouplers. Make sure the device is connected to the AC power - board has a own power supply. AC High Voltage is **VERY DANGEROUS**. It can cause more serious issues such as *Electrical injury* of *death*. **Don't be stupid.**. 

### Features
  * Read access to all Input and Holding registers
  * Write access to Manufacturer recomended Holding registers
    * Special build flag for full write access to **all** Holding registers
    * Write protection mechanism in case of full write access 
  * Callback on power alarm state
  * Communication with **device Modbus RTU Address**
  * CRC16_MODBUS checking out-of-the-box
  * Communication errors statistics

## Installation
Download the ZIP of this repository and install it manually. The guide how to do that [can be found here](https://www.arduino.cc/en/guide/libraries)

## Other
If you want to dig in into this device please read my remarks in docs folder. There are (TODO) hardware modding tips and complete communication frames explanations for nerds :-)

***
