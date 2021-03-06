# Moisture Probe CubeCell HTCC-AB01
Single Board Computer used is [CubeCell](https://heltec.org/project/htcc-ab01/) HTCC-AB01 from [Heltec Automation](https://heltec.org)

## Project
_(This project has been initiated by a need to develop low cost probes using the LoRaWAN protocol)_
For the OneWire probe DS18B20, the library used by default <OneWire.h> and <DallasTemperature.h> seems to be a problem
for several people as for me and I opted to control a GPIO to manage read/write operations on OneWire bus.

## Solution and future upgrade
This solution is one among several and it was not possible without oscilloscope to get correct timing.
It lacks somme essentials functions in the project to check the CRC8 code and it wasn't for lack of not trying. My function
does not retrieve the correct CRC8 sended by the sensor DS18B20. It is obviously a failure of my own read of the
datasheet provided by [Maxim](https://www.maximintegrated.com/en.html).

_**upgrade awaited:**_
* A function to calculate the CRC8 code sended by the DS18B20 sensor.
* It will be necesary to activate interruptions of the PSoC4 timers.

## Illustrations

## IDE and package
IDE: The project is an open source using [Arduino](https://www.arduino.cc/) Integrated Development Environment.
Package: A link from [helium](https://developer.helium.com/devices/arduino-quickstart/heltec-cubecell-board) which explains how to install the package from Heltec Automation.

## Libraries and documentations
* ADS1115: [Adafruit_ADS1015](https://github.com/adafruit/Adafruit_ADS1X15).
* [Documents](https://heltec.org/project/htcc-ab01/) from Heltec Automation.

## Authors
* Jean-louis Druilhe, email: jean-louis.druilhe@univ-tlse3.fr

## License
[MIT License](https://opensource.org/licenses/MIT)