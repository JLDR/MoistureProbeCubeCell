{\rtf1\ansi\ansicpg1252\deff0\nouicompat{\fonttbl{\f0\fnil\fcharset0 Consolas;}}
{\*\generator Riched20 10.0.18362}\viewkind4\uc1 
\pard\sl276\slmult1\f0\fs22\lang12 #Moisture Probe CubeCell HTCC-AB01\par
Single Board Computer used is [CubeCell](https://heltec.org/project/htcc-ab01/) HTCC-AB01 from [Heltec Automation](https://heltec.org)\par
\par
##Project\par
_(This project has been initiated by a need to develop low cost probes using the LoRaWAN protocol)_\par
For the OneWire probe DS18B20, the library used by default <OneWire.h> and <DallasTemperature.h> seems to be a problem\par
for several people as for me and I opted to control a GPIO to manage read/write operations on OneWire bus.\par
\par
##Solution and future upgrade\par
This solution is one among several and it was not possible without oscilloscope to get correct timing.\par
It lacks somme essentials functions in the project to check the CRC8 code and it wasn't for lack of not trying. My function\par
does not retrieve the correct CRC8 sended by the sensor DS18B20. It is obviously a failure of my own read of the\par
datasheet provided by [Maxim](https://www.maximintegrated.com/en.html).\par
\par
_**upgrade awaited:**_\par
* A function to calculate the CRC8 code sended by the DS18B20 sensor.\par
* It will be necesary to activate interruptions of the PSoC4 timers.\par
\par
##Illustrations\par
\par
##IDE and package\par
IDE: The project is an open source using [Arduino](https://www.arduino.cc/) Integrated Development Environment. \par
Package: A link from [helium](https://developer.helium.com/devices/arduino-quickstart/heltec-cubecell-board) which explains how to install the package from Heltec Automation.\par
\par
##Libraries and documentations\par
* ADS1115: [Adafruit_ADS1015](https://github.com/adafruit/Adafruit_ADS1X15).\par
* [Documents](https://heltec.org/project/htcc-ab01/) from Heltec Automation.\par
\par
##Authors\par
* Jean-louis Druilhe, email: jean-louis.druilhe@univ-tlse3.fr\par
\par
##License\par
[MIT License](https://opensource.org/licenses/MIT)\par
\par
}
 