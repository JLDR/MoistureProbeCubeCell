/* ******************************************************************************************** */
/* The whole of functions involved with this LoRaWAN application and also thanks to the good    */
/* quality of the single board computer provided by Heltec Automation.                          */
/* ******************************************************************************************** */
#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_            1

#include        "LoRaWan_APP.h"
#include        "Arduino.h"
#include        <Adafruit_ADS1015.h>
#include        <Wire.h>
#include        <string.h>                  /* traitement des chaînes de caractères */
#include        <stdint.h>
#include        <stdio.h>                   /* sprintf */
#include        <stdlib.h>                  /* nécessaire pour la fonction atof */
#include        <Adafruit_Sensor.h>

/*                                  Preprocessor directives                                     */  
/* ******************************************************************************************** */
/* https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/quick_start.html            */
/* set LoraWan_RGB to Active :                                                                  */       
/*                - RGB red means sending;                                                      */
/*                - RGB purple means joined done;                                               */
/*                - RGB blue means First RxWindow (RxWindow1);                                  */
/*                - RGB yellow means Second RxWindow (RxWindow2);                               */
/*                - RGB green means received done;                                              */
/* ******************************************************************************************** */
//#ifndef LoraWan_RGB                         // Compiler directive which is encoutered in files of the library
//  #define       LoraWan_RGB
//#endif
//#define         DataChecking                // to display at terminal some results
//#define         Short_FrameON               // if we want to shrink a float number as 2 bytes with a int16_t
//#define         ControlCommand
/* ******************************************************************************************** */

/* CONSTANTES */
#define         DS18S20Pin_GPIO0            GPIO0         // first probe DS18B20 (Probe1) connected on GPIO0 and tied to Binder Connector Number 1
#define         DS18S20Pin_GPIO1            GPIO1         // second probe DS18B20 (Probe3) connected on GPIO1 and tied to Binder Connector Number 2
#define         DS18S20Pin_GPIO2            GPIO2         // third probe DS18B20 (Probe4) connected on GPIO2 and tied to Binder Connector Number 3
#define         Channel_ADC                 1
#define         valMax                      32767
#define         AdcFullScale                4096          // 12 bits resolution (https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/frequently_asked_questions.html)
#define         AnalogReference             1.2
#define         VDD_nominal                 3.3
#define         gain2_048                   2.048
#define         gain4_096                   4.096
#define         MAX_NbrBYTES                50            // maximum number of bytes transmitted (array dimension)
#define         RegLin                      -0.0078       // relation pente 
#define         ordonnee_origine            185           // 185 % soit l'ordonnée pour un signal à l'entrée de l'ADC de 0,0 V

#define         Channel_ADC                 1
#define         DS18S20MODEL                0x10
#define         DS18B20MODEL                0x28
#define         DS1822MODEL                 0x22

// Scratchpad locations
#define         TEMP_LSB                    0
#define         TEMP_MSB                    1
#define         HIGH_ALARM_TEMP             2
#define         LOW_ALARM_TEMP              3
#define         CONFIGURATION               4
#define         INTERNAL_BYTE               5
#define         COUNT_REMAIN                6
#define         COUNT_PER_C                 7
#define         SCRATCHPAD_CRC              8
// Device resolution
#define         TEMP_9_BIT                  0x1F          //  9 bit
#define         TEMP_10_BIT                 0x3F          // 10 bit
#define         TEMP_11_BIT                 0x5F          // 11 bit
#define         TEMP_12_BIT                 0x7F          // 12 bit

#define         LF                          0x0A          // '\n'
#define         CR                          0x0D          // '\r'
#define         Null                        '\0'
#define         Nbr_CarBuff                 40

#define         RF_FREQUENCY                868000000     // Hz
#define         TX_OUTPUT_POWER             14            // dBm
#define         LORA_BANDWIDTH              0             // [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define         LORA_SPREADING_FACTOR       8             // [SF7..SF12]
#define         LORA_CODINGRATE             4             // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define         LORA_PREAMBLE_LENGTH        8             // Same for Tx and Rx
#define         LORA_SYMBOL_TIMEOUT         0             // Symbols
#define         LORA_FIX_LENGTH_PAYLOAD_ON  false
#define         LORA_IQ_INVERSION_ON        false
#define         RX_TIMEOUT_VALUE            1000
#define         BUFFER_SIZE                 30            // Define the payload size here
#define         Sign_Mask                   0x80000000    // for float numbers defined with 4 bytes


/********************************************** Types prédéfinis **********************************************/

typedef union flottant {
  float value;
  uint8_t byte_value[4];    // little-endian representation
} MyFloat_t;

typedef struct UI64 {
  uint64_t Value;
  uint32_t MSDoubleWord;
  uint32_t LSDoubleWord;
} UI64_t;

typedef enum Probes {
  Probe1 = 1,
  Probe2,
  Probe3,
  Probe4
} Probe_t;

typedef enum Resolution {
  ResConv9bits  = 0x1F,         //  9 bit
  ResConv10bits = 0x3F,         // 10 bit
  ResConv11bits = 0x5F,         // 11 bit
  ResConv12bits = 0x7F          // 12 bit
} Resolution_t;

typedef enum ROM_Commands : uint8_t {
  Search_ROM  = 0xF0,
  Read_ROM = 0x33,
  Match_ROM = 0x55,
  Skip_ROM = 0xCC,
  Alarm_Search = 0xEC
} ROM_Commands_t;

typedef enum Function_Commands {
  Convert_T  = 0x44,
  Write_Scratchpad = 0x4E,
  Read_Scratchpad = 0xBE,
  Copy_Scratchpad = 0x48,
  Recall_E2 = 0xB8,
  ReadPowerSupply = 0xB4
} Function_Commands_t;

/********************************* prototyping of functions *********************************/
void prepareTxFrame(void);
void MeasuresFrame(void);
void scanI2Cbus(void);
void ADCStartUp(adsGain_t);
void separateur(uint8_t, char);
void StoreAddressRead_In_Array2dimensions(Probe_t);
void Load_DeviceAddress(Probe_t);
void DeviceIdentifier(uint8_t);
void AfficheAdresseCapteur(Probe_t);
void AcquireHumidity(Probe_t);
void DisplayBatteryVoltage(void);
void WriteByte(uint8_t, Probe_t);
uint8_t ReadByte(Probe_t);
void SendRomCommand(ROM_Commands_t, Probe_t);
void DisplayAddressProbe(Probe_t);
bool ResetOneWire(Probe_t);
uint8_t IdentifyAddresses(void);
void SendFunctionCommand(Function_Commands_t, Probe_t);
bool IsConversionComplete(Probe_t);
void SelectOneDevice(Probe_t, uint8_t *);
float TemperatureMeasure(Probe_t, uint8_t *);
void AcquireTempMeasureAndDisplay(Probe_t);
void ConvFloatToString(float, uint8_t, uint8_t, char *);
uint8_t ConvertUint32ToASCIIChar(char *, uint32_t);
void OneWireReset(String);





#endif      /* FUNCTIONS_H_ */
