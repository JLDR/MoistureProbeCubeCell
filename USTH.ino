/* Heltec Automation test example and communication LoRa
 * using LoRaWAN protocol. The project is designed to be entrusted to University of Science and Technology of Hanoi
 * https://www.usth.edu.vn/en/
 * Class A device
 * LoRaWAN application on server : https://lorawan.univ-tlse3.fr/admin#/dashboard
 * Authentification mode : OTAA
 * Single Board Computer used from: https://heltec.org
 * ##################################################################################################################################
 * PROBLEMATIC : The I2C bus being common and shared by all sensors, it will not possible to identify positions of humidity probes  #                           
 * even using their physical addresses.                                                                                             #
 * So we need to call a function using IDE terminal to get the addresses of each probes with the following relationship :           #
 * Probe 1 which is made of one DS18B20 sensor associated with ADS1115 which I2C address is 0x48.                                   #
 * Probe 2 must not have to be choosen because the address connexion tied to VDD of the ADS1115 disturbs the DS18B20 sensor.        #
 * Probe 3 is associated with I2C address 0x4A.                                                                                     #
 * Probe 4 is associated with I2C address 0x4B.                                                                                     #
 * So for each I2C address, a function has to allow a lecture of the uint64_t address of each DS18B20 sensor.                       #
 * The only physical connection which is fixed is the wiring of the DS18B20 sensor to a dedicated GPIO of the HTCC-AB01 card.       #
 * So the first action for the program is to recognize the DS18B20 sensors on each BINDER connectors reading their addresses.       #
 * When all addresses of each DS1B20 sensor are known, it is not possible to identify the humidity sensor except if we write for    #
 * each probe the specific I2C address of the associated converter.                                                                 #
 * ##################################################################################################################################
 * GPIO7 is used to measure the battery's voltage and is equivalent as ADC_CTL. To apply the Vbat potential at the ADC input, GPIO7 /
 * has to be held to a low logic level. The potential at the input of the ADC must not exceed 2V4.                                  /
 * GPIO6 is used to deliver the regulated supply of 3V3 to other external devices. When GPIO6 is held at low logic level the Vext   / 
 * connexion can supply the power of an external device.                                                                            /
 */
#include "Functions.h"

char txPacket[BUFFER_SIZE];
char rxPacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;   // Ce type est déclaré dans radio.h et correspond à une structure de pointeurs de fonctions
/* LoraWan region, select in arduino IDE tools */
//LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;       // LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;       // ou constante de l'IDE liée à la définition du fichier boards.txt (menu.LORAWAN_REGION)
/* LoraWan Class, Class A and Class C are supported */
DeviceClass_t loraWanClass = CLASS_A;                       // DeviceClass_t loraWanClass = LORAWAN_CLASS; (type défini dans LoRaMac.h)
/* OTAA or ABP */
//DeviceClass_t loraWanClass = LORAWAN_CLASS;                 // ou constante de l'IDE liée à la définition du fichier boards.txt (menu.LORAWAN_CLASS)
bool overTheAirActivation = LORAWAN_NETMODE;                // ou bool overTheAirActivation = true; 
/*Adaptive Data Rate enable */
bool loraWanAdr = LORAWAN_ADR;                              // ou bool loraWanAdr = true; (bool loraWanAdr déclaré dans LoRaWan_APP.h en externe)
/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;                    // bool isTxConfirmed declared in LoRaWan_APP.h as external
/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;                         // bool keepNet declared in LoRaWan_APP.h en external
/* Application port */
uint8_t appPort = 224;                                      // what you want but strictly inferoir than 225, uint8_t appPort declared in LoRaWan_APP.h
/* the application data transmission duty cycle.  value in [ms]. */
uint32_t appTxDutyCycle = 240000;                           // => 4 minutes (déclaré aussi dans LoRaWan_APP.h)
//uint32_t appTxDutyCycle = 300000;                           // => 5 minutes (déclaré aussi dans LoRaWan_APP.h)
/* Number of trials to transmit the frame, if the LoRaMAC layer did not receive an acknowledgment.
 * The MAC performs a datarate adaptation, according to the LoRaWAN Specification V1.0.2, chapter 18.4, according to the following table:
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment */
uint8_t confirmedNbTrials = 8;

/* OTAA mode*/
// This Eui must be in little-endian format, so least-significant-byte first. 
// When copying an EUI from ttnctl output, this means to reverse the bytes.
uint8_t devEui[8] PROGMEM = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};             // your DevEUI provided by your server LoRaWAN application
uint8_t appEui[8] PROGMEM = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};             // your AppEUI provided by your server LoRaWAN application
// This key should be in big endian format (or, since it is not really a number but a block of memory, endianness does not really apply).
uint8_t appKey[16] PROGMEM = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};  // your application key
/* ABP mode (doit être mentionné obligatoirement même si le mode OTAA est choisi) */
uint8_t nwkSKey[] PROGMEM = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};   // your network session key
uint8_t appSKey[] PROGMEM = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};   // your application session key
uint32_t devAddr = (uint32_t)0x6789ABCD;                                                                                        // the identifier of your device

/* Variables globales et propres à ce fichier */
uint8_t                 Nbr_probes;
uint8_t                 NbrBytes;                 // number of bytes transmitted by terminal

/* External variables */
//extern bool             CheckDS18B20Probe1, CheckDS18B20Probe3, CheckDS18B20Probe4;

/* instances de classes */

/****************************************************************************************************/
void setup() {
  boardInitMcu();                             // function defined in header file board.h and definition is in asr_board.c
  Serial.begin(115200);
  Serial.println(F("Startup Heltec CubeCell HTCC-AB01 Dev-Board"));
  /* I2C bus */
  Wire.begin();                               // activates the ADC initialization
  scanI2Cbus();
  ADCStartUp(GAIN_ONE);
  DisplayBatteryVoltage();                    // problem
  /* OneWire bus */
  Nbr_probes = IdentifyAddresses();
  separateur(80, '-');
  Serial.print(F("Number of temperature probes DS18B20 found: "));
  Serial.println(Nbr_probes, DEC);
  separateur(80, '*');
  NbrBytes = 0;
  deviceState = DEVICE_STATE_INIT;            // constant defined in LoRaWan_APP.cpp
  LoRaWAN.ifskipjoin();                       // object from LoRaWanClass defined in LoRaWan_APP.h
                                              // method ifskipjoin() defined in LoRaWan_APP.cpp
}

void loop() {
  switch(deviceState) {
    case DEVICE_STATE_INIT:                         // AT_Command.h
    {
      printDevParam();
      LoRaWAN.init(loraWanClass, loraWanRegion);    // object defined in LoRaWan_APP.c
      deviceState = DEVICE_STATE_JOIN;
      break;
    }       // to limit the scope of variables which have been changed between curly brackets
    case DEVICE_STATE_JOIN:
    {
      LoRaWAN.join();               // méthode dans LoRaWan_APP.cpp
      break;                        // deux options possibles en fonction du résultat du test de la méthode LoRaMacMlmeRequest() 
    }                               // DEVICE_STATE_SLEEP or DEVICE_STATE_CYCLE
    case DEVICE_STATE_SEND:
    {
      #ifdef Short_FrameON
        prepareTxFrame();
      #else
        MeasuresFrame();
      #endif
      LoRaWAN.send();
      deviceState = DEVICE_STATE_CYCLE;
      break;
    }
    case DEVICE_STATE_CYCLE:
    {
      // Schedule next packet transmission
      txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
      LoRaWAN.sleep();
      break;
    }
    default:
    {
      deviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}







/* END OF FILE */
