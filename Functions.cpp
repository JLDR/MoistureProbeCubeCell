
#include "Functions.h"

extern char                 txPacket[BUFFER_SIZE];
extern char                 rxPacket[BUFFER_SIZE];
extern DeviceClass_t        loraWanClass;
extern LoRaMacRegion_t      loraWanRegion;
extern uint32_t             appTxDutyCycle;
extern bool                 overTheAirActivation;
extern uint8_t              i, j, k, l, m, n, p, t, q;

/************************ Variables locales ************************/
char                        TabAsciiFunction[20];
uint8_t                     BigEndianFormat[4];
char                        TempArray[20];
uint8_t                     FrameToTransmit[MAX_NbrBYTES];
uint16_t                    adc1, adc3, adc4;
float                       HumRH1, HumRH3, HumRH4;
uint16_t                    HumShrink1, HumShrink3, HumShrink4;
float                       Ds18B20Temp1, Ds18B20Temp3, Ds18B20Temp4;
int16_t                     tempProbe1, tempProbe3, tempProbe4;           // Two's complement
uint16_t                    NbrIncrements;
float                       BatteryVoltage;
uint16_t                    BatShrink;
int16_t                     Nbr_mV_Integer;
uint8_t                     Device_addresses[4][8];   // In former version this array stores each 64-bits addresses
uint8_t                     Device_Address[8];
static uint8_t              Probe1_Address[8];
static uint8_t              Probe3_Address[8];
static uint8_t              Probe4_Address[8];
uint8_t                     SRAM_ScratchPad[9];       // dimension 9 depends of number of DS18B20 registers (common for all probes)
static uint8_t              DegreeLabel[] = "Temp: ";
static uint8_t              HumidityLabel[] = "Hum: ";
bool                        CheckDS18B20Probe1, CheckDS18B20Probe3, CheckDS18B20Probe4;
bool                        Condition;
uint8_t                     scratch_8bitsFunc;
uint16_t                    scratch_16bitsFunc;
uint32_t                    scratch_32bitsFunc;
uint64_t                    scratch_64bitsFunc;

/* class objects */
Adafruit_ADS1115 ads1115_Probe1(0x48);          // construct an ads1115 at address 0x48 (ADDR tied to GND)
Adafruit_ADS1115 ads1115_Probe3(0x4A);          // construct an ads1115 at address 0x4A (ADDR tied to SDA)
Adafruit_ADS1115 ads1115_Probe4(0x4B);          // construct an ads1115 at address 0x4B (ADDR tied to SCL)

/********************************************************************************************************************/
/* Prepares the payload of the frame.                                                                               */
/* appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h". appData[] array is defined in   */
/* LoRaWan_APP.h and appDataSize is a variable of LoRaWan_APP.h also.                                               */
/* appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.                                                              */
/* if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.                   */
/* if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.  */
/* for example, if use REGION_EU868, the max value for different DR can be found in MaxPayloadOfDatarateEU868       */
/* refer to DataratesEU868 and BandwidthsEU868 in "RegionEU868.h".                                                  */
/* static const uint8_t MaxPayloadOfDatarateEU868[] = {51, 51, 51, 115, 242, 242, 242, 242};                        */
/* static const uint32_t BandwidthsEU868[] = {125000, 125000, 125000, 125000, 125000, 125000, 250000, 0};           */
/* static const uint8_t DataratesEU868[]  = {12, 11, 10, 9, 8, 7, 7, 50};                                           */
/* appData is an uint8_t array declared in LoRaWan_APP.cpp (uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];)            */
/* Vext or GPIO6 = LOW => involves for MOSFET AO7801 a conducting channel between source and drain. So using        */
/* schematic, we can observe that the control of this MOSFET allows the supply of the RGB Led.                      */
/* frame: HumShrink1(uint16_t), '%', tempProbe1(int16_t), 'C', HumShrink3(uint16_t), '%', tempProbe3(int16_t), 'C'  */
/*        HumShrink4(uint16_t), '%', tempProbe4(int16_t), 'C', NbrIncrements(uin16_t), 'V' => 21 bytes              */
/* Frame example: 05462501294303C625012A43043A25012B43000056                                                        */
/********************************************************************************************************************/
void prepareTxFrame(void) {
  MyFloat_t ddpADC1, ddpADC3, ddpADC4;
  pinMode(Vext, OUTPUT);            // GPIO6 which is not mentioned on schematic, here Vext should have been mentioned as Vext_CTL
  digitalWrite(Vext, LOW);          // control MOSFET canal P to allow VDD to be applied at output Vext to supply the RGB LED
  
  memset(appData, '\0', sizeof(appData));                           // LoRaWan_APP.cpp
  appDataSize = 0;                                                  // will be increased
  
  if (CheckDS18B20Probe1 == true) {
    adc1 = ads1115_Probe1.readADC_SingleEnded(Channel_ADC);         // send back an uint16_t
    ddpADC1.value = ((float)adc1 / (float)valMax) * gain2_048;      // integer values to be cheched
    HumRH1 = (RegLin * (float)adc1) + (float)ordonnee_origine;      // y = a.x + b with a < 0 et b = 185 %RH
    HumShrink1 = (uint16_t)(HumRH1 * 100.0);
    scratch_8bitsFunc = (uint8_t)(HumShrink1 >> 8);
    appData[appDataSize++] = scratch_8bitsFunc;
    appData[appDataSize++] = (uint8_t)HumShrink1;
    appData[appDataSize++] = (uint8_t)('%');
    SendFunctionCommand(Convert_T, Probe1);                         // Ds18B20Temp1 and tempProbe1 (int16_t) updated
    scratch_8bitsFunc = (uint8_t)(tempProbe1 >> 8);                 // Measure = (float)tempProbe1 * 0.0625;
    appData[appDataSize++] = scratch_8bitsFunc;
    appData[appDataSize++] = (uint8_t)tempProbe1;
    appData[appDataSize++] = (uint8_t)('C');
  }
  
  if (CheckDS18B20Probe3 == true) {
    adc3 = ads1115_Probe3.readADC_SingleEnded(Channel_ADC);         // send back an uint16_t
    ddpADC3.value = ((float)adc3 / (float)valMax) * gain2_048;      // integer values to be cheched
    HumRH3 = (RegLin * (float)adc3) + (float)ordonnee_origine;      // y = a.x + b with a < 0 et b = 185 %RH
    HumShrink3 = (uint16_t)(HumRH3 * 100.0);
    scratch_8bitsFunc = (uint8_t)(HumShrink3 >> 8);
    appData[appDataSize++] = scratch_8bitsFunc;
    appData[appDataSize++] = (uint8_t)HumShrink3;
    appData[appDataSize++] = (uint8_t)('%');
    SendFunctionCommand(Convert_T, Probe3);                         // Ds18B20Temp1 and tempProbe1 (int16_t) updated
    scratch_8bitsFunc = (uint8_t)(tempProbe3 >> 8);                 // Measure = (float)tempProbe1 * 0.0625;
    appData[appDataSize++] = scratch_8bitsFunc;
    appData[appDataSize++] = (uint8_t)tempProbe3;
    appData[appDataSize++] = (uint8_t)('C');
  }
  
  if (CheckDS18B20Probe4 == true) {
    adc4 = ads1115_Probe4.readADC_SingleEnded(Channel_ADC);         // send back an uint16_t
    ddpADC4.value = ((float)adc4 / (float)valMax) * gain2_048;      // integer values to be cheched
    HumRH4 = (RegLin * (float)adc4) + (float)ordonnee_origine;      // y = a.x + b with a < 0 et b = 185 %RH
    HumShrink4 = (uint16_t)(HumRH4 * 100.0);
    scratch_8bitsFunc = (uint8_t)(HumShrink4 >> 8);
    appData[appDataSize++] = scratch_8bitsFunc;
    appData[appDataSize++] = (uint8_t)HumShrink4;
    appData[appDataSize++] = (uint8_t)('%');
    SendFunctionCommand(Convert_T, Probe4);                         // Ds18B20Temp1 and tempProbe1 (int16_t) updated
    scratch_8bitsFunc = (uint8_t)(tempProbe4 >> 8);                 // Measure = (float)tempProbe1 * 0.0625;
    appData[appDataSize++] = scratch_8bitsFunc;
    appData[appDataSize++] = (uint8_t)tempProbe4;
    appData[appDataSize++] = (uint8_t)('C');
  }
  
  digitalWrite(Vext, HIGH);                                         // VDD is not applied to Vext so LED RGB is off
  NbrIncrements = getBatteryVoltage();                              // method defined in LoRaWan_APP.cpp (uint16_t returned)
  BatteryVoltage = ((float)(NbrIncrements / AdcFullScale)) * AnalogReference;
  BatShrink = (uint16_t)(BatteryVoltage * 100.0);
  scratch_8bitsFunc = (uint8_t)(BatShrink >> 8);
  appData[appDataSize++] = scratch_8bitsFunc;
  appData[appDataSize++] = (uint8_t)BatShrink;
  appData[appDataSize++] = (uint8_t)('V');
  
  #ifdef DataChecking
    separateur(100, '_');
    // Probe1 is the couple ADS1115 (Humidity) with I2C address 0x48 associated with sensors_N1 (DS18B20)
    Serial.print(F("Integer returned (uint6_t) from ADC1 : "));
    Serial.println(adc1, DEC);
    Serial.print(F("Corresponding voltage : "));
    Serial.print(ddpADC1.value, 3); Serial.println(" V");
    Serial.print(F("Relative humidity from probe 1 : "));
    Serial.print(HumRH1, 2); Serial.println(" %RH");
    Serial.print(F("Temperature from DS18B20 sensor associated to the \"humidity\" address 0x48 : "));
    Serial.print(Ds18B20Temp1, 2); Serial.println(" °C");
    separateur(20, '*');
    // Probe3 is the couple ADS1115 (Humidity) with I2C address 0x4A associated with sensors_N3 (DS18B20)
    Serial.print(F("Integer returned (uint6_t) from ADC3 : "));
    Serial.println(adc3, DEC);
    Serial.print(F("Corresponding voltage : "));
    Serial.print(ddpADC3.value, 3); Serial.println(" V");
    Serial.print(F("Relative humidity from probe 3 : "));
    Serial.print(HumRH3, 2); Serial.println(" %RH");
    Serial.print(F("Temperature from DS18B20 sensor associated to the \"humidity\" address 0x4A : "));
    Serial.print(Ds18B20Temp3, 2); Serial.println(" °C");
    separateur(20, '*');
    // Probe4 is the couple ADS1115 (Humidity) with I2C address 0x4B associated with sensors_N4 (DS18B20)
    Serial.print(F("Integer returned (uint6_t) from ADC4 : "));
    Serial.println(adc4, DEC);
    Serial.print(F("Corresponding voltage : "));
    Serial.print(ddpADC4.value, 3); Serial.println(" V");
    Serial.print(F("Relative humidity from probe 4 : "));
    Serial.print(HumRH4, 2); Serial.println(" %RH");
    Serial.print(F("Temperature from DS18B20 sensor associated to the \"humidity\" address 0x4B : "));
    Serial.print(Ds18B20Temp4, 2); Serial.println(" °C");
    separateur(20, '*');
    Serial.print(F("Battery voltage : "));
    Serial.println(BatteryVoltage, 2); Serial.println(" V");
    separateur(100, '_');
  #endif
}
/********************************************************************************************************************/
/* Frame packet using ASCII format. This frame is also longer than the previous function but the content will be    */
/* interpreted more easily on LoRaWAN application installed on server. Floats are with two decimals.                */
/* frame: AsciiHumidity1(5bytes)), '%', AsciiDs18B20Temp1(5 bytes), 'C', AsciiHumidity2(5 bytes), '%',              */
/*        AsciiDs18B20Temp2(5 bytes), 'C', AsciiHumidity3(5bytes)), '%', AsciiDs18B20Temp3(5 bytes), 'C',           */
/*        BatteryVolatge (4 bytes), 'V'                                                                             */
/* Frame example : 31332E36352531372E343343392E38312531372E36324331302E37382531372E363243312E313356                 */
/********************************************************************************************************************/
void MeasuresFrame(void) {
  uint8_t k;
  MyFloat_t ddpADC1, ddpADC3, ddpADC4;
  pinMode(Vext, OUTPUT);            // GPIO6 which is not mentioned on schematic, here Vext should have been mentioned as Vext_CTL
  digitalWrite(Vext, LOW);          // control MOSFET canal P to allow VDD to be applied at output Vext to supply the RGB LED
  
  memset(appData, '\0', sizeof(appData));       // LoRaWan_APP.cpp (uint8_t appData[LORAWAN_APP_DATA_MAX_SIZE];)
  appDataSize = 0;                              // will be increased (attribute from LoRaWan_APP.cpp)
  
  if (CheckDS18B20Probe1 == true) {
    adc1 = ads1115_Probe1.readADC_SingleEnded(Channel_ADC);         // send back an uint16_t
    ddpADC1.value = ((float)adc1 / (float)valMax) * gain2_048;      // integer values to be cheched
    HumRH1 = (RegLin * (float)adc1) + (float)ordonnee_origine;      // y = a.x + b with a < 0 et b = 185 %RH
    if (HumRH1 < 10.0) {
      ConvFloatToString(HumRH1, 4, 2, &TabAsciiFunction[0]);
      for (k = 0; k < 4; k++) appData[appDataSize++] = TabAsciiFunction[k];
    } else {
      ConvFloatToString(HumRH1, 5, 2, &TabAsciiFunction[0]);
      for (k = 0; k < 5; k++) appData[appDataSize++] = TabAsciiFunction[k];
    }
    appData[appDataSize++] = (uint8_t)('%');
    SendFunctionCommand(Convert_T, Probe1);                         // Ds18B20Temp1 and tempProbe1 (int16_t) updated
    ConvFloatToString(Ds18B20Temp1, 5, 2, &TabAsciiFunction[0]);
    for (k = 0; k < 5; k++) appData[appDataSize++] = TabAsciiFunction[k];
    appData[appDataSize++] = (uint8_t)('C');
  }
  
  if (CheckDS18B20Probe3 == true) {
    adc3 = ads1115_Probe3.readADC_SingleEnded(Channel_ADC);         // send back an uint16_t
    ddpADC3.value = ((float)adc3 / (float)valMax) * gain2_048;      // integer values to be cheched
    HumRH3 = (RegLin * (float)adc3) + (float)ordonnee_origine;      // y = a.x + b with a < 0 et b = 185 %RH
    if (HumRH3 < 10.0) {
      ConvFloatToString(HumRH3, 4, 2, &TabAsciiFunction[0]);
      for (k = 0; k < 4; k++) appData[appDataSize++] = TabAsciiFunction[k];
    } else {
      ConvFloatToString(HumRH3, 5, 2, &TabAsciiFunction[0]);
      for (k = 0; k < 5; k++) appData[appDataSize++] = TabAsciiFunction[k];
    }
    appData[appDataSize++] = (uint8_t)('%');
    SendFunctionCommand(Convert_T, Probe3);                         // Ds18B20Temp1 and tempProbe1 (int16_t) updated
    ConvFloatToString(Ds18B20Temp3, 5, 2, &TabAsciiFunction[0]);
    for (k = 0; k < 5; k++) appData[appDataSize++] = TabAsciiFunction[k];
    appData[appDataSize++] = (uint8_t)('C');
  }
  
  if (CheckDS18B20Probe4 == true) {
    adc4 = ads1115_Probe4.readADC_SingleEnded(Channel_ADC);         // send back an uint16_t
    ddpADC4.value = ((float)adc4 / (float)valMax) * gain2_048;      // integer values to be cheched
    HumRH4 = (RegLin * (float)adc4) + (float)ordonnee_origine;      // y = a.x + b with a < 0 et b = 185 %RH
    if (HumRH4 < 10.0) {
      ConvFloatToString(HumRH4, 4, 2, &TabAsciiFunction[0]);
      for (k = 0; k < 4; k++) appData[appDataSize++] = TabAsciiFunction[k];
    } else {
      ConvFloatToString(HumRH4, 5, 2, &TabAsciiFunction[0]);
      for (k = 0; k < 5; k++) appData[appDataSize++] = TabAsciiFunction[k];
    }
    appData[appDataSize++] = (uint8_t)('%');
    SendFunctionCommand(Convert_T, Probe4);                         // Ds18B20Temp1 and tempProbe1 (int16_t) updated
    ConvFloatToString(Ds18B20Temp4, 5, 2, &TabAsciiFunction[0]);
    for (k = 0; k < 5; k++) appData[appDataSize++] = TabAsciiFunction[k];
    appData[appDataSize++] = (uint8_t)('C');
  }
  
  digitalWrite(Vext, HIGH);                   // to supply the RGB LED (Vext defined in ASR_Arduino.h)
  NbrIncrements = getBatteryVoltage();
  BatteryVoltage = ((float)NbrIncrements / (float)AdcFullScale) * AnalogReference;
  ConvFloatToString(BatteryVoltage, 4, 2, &TabAsciiFunction[0]);
  for (k = 0; k < 4; k++) appData[appDataSize++] = TabAsciiFunction[k];
  appData[appDataSize++] = (uint8_t)('V');
}
/**********************************************************************************************************************************/
/* Polling function to check the presence of I2C devices.                                                                         */
/**********************************************************************************************************************************/
void scanI2Cbus(void) {                               // Scan for all I2C devices
  uint8_t NbrDevice = 0;
  uint8_t k;
  Serial.println(F("---------------------------"));
  Serial.println(F("Starting  I2C scan..."));
  for (k = 1; k < 128; k++) {
    Wire.beginTransmission(k);                        // If true, endTransmission() sends a stop message after transmission, releasing the I2C bus
    if (Wire.endTransmission() == 0) {                // if device is present
      NbrDevice++;                                    // number of device increased
      Serial.print(F("I2C used channel 0x"));
      if (k < 16) Serial.print("0");
      Serial.println(k, HEX);                         // hexadecimal address        
    }
  }
  Serial.println(F("SCAN COMPLETE"));
  Serial.print(F("I2C devices encountered: "));
  Serial.println(NbrDevice, DEC);
  Serial.println(F("---------------------------"));
}
/****************************************************************************************************/
/* identification of probes on I2C bus from which instance are declared.                            */
/* With 5 Volts power supply 5 V: ads1115.setGain(GAIN_ONE);  // 1x gain +/- 4.096V 1 bit = 2mV     */
/* With 3.3 Volts power supply: ads1115.setGain(GAIN_TWO);                                          */
/****************************************************************************************************/
void ADCStartUp(adsGain_t ADC_Gain) {
  //ads1115_Probe1.begin();                     // equivalent as Wire.begin(); => no necessary
  ads1115_Probe1.setGain(ADC_Gain);           // 2x gain +/- 2.048 Volts LSB = 62,5 µV
  ads1115_Probe3.setGain(ADC_Gain);           // GAIN_ONE : 1x gain +/- 4.096V
  ads1115_Probe4.setGain(ADC_Gain);
  AcquireHumidity(Probe1);
  AcquireHumidity(Probe3);
  AcquireHumidity(Probe4);
}
/****************************************************************************************************/
/* Divider                                                                                          */
/****************************************************************************************************/
void separateur(uint8_t nbr_carac, char caract) {
  for (uint8_t Inc = 0; Inc < nbr_carac; Inc++) {
    Serial.print(caract);
  }
  Serial.println();
}
/****************************************************************************************************/
/* Function to store an address found by polling method in an array of two-dimension. The parameter */
/* passed to argument is a type defined in Functions.h                                              */
/****************************************************************************************************/
void StoreAddressRead_In_Array2dimensions(Probe_t MyProbe) {
  uint8_t k;
  uint8_t *ptrTwoDim;
  uint8_t *ptrOneDim;
  switch (MyProbe) {
    case Probe1:
      ptrTwoDim = &Device_addresses[0][0];
      ptrOneDim = &Probe1_Address[0];
      break;
    case Probe3:
      ptrTwoDim = &Device_addresses[2][0];
      ptrOneDim = &Probe3_Address[0];
      break;
    case Probe4:
      ptrTwoDim = &Device_addresses[3][0];
      ptrOneDim = &Probe4_Address[0];
      break;
  }
  
  #ifdef DataChecking
    separateur(80, '-');              // ends with a new line
    Serial.print(F("Content of the unidimensional array: "));
    for (k = 0; k < 8; k++) {
      if (*ptrOneDim > 9) Serial.print(*ptrOneDim, HEX);
      else {
        Serial.print('0');
        Serial.print(*ptrOneDim);
      }
      Serial.print(' ');
      *(ptrTwoDim++) = *(ptrOneDim++);
    }
    Serial.print(F("\nContent of the two-dimensional array: "));
    k = (uint8_t)MyProbe - 1;
    ptrTwoDim = &Device_addresses[k][0];
    for (k = 0; k < 8; k++) {
      if (*ptrTwoDim > 9) Serial.print(*ptrTwoDim, HEX);
      else {
        Serial.print('0');
        Serial.print(*ptrTwoDim);
      }
      Serial.print(' ');
      ptrTwoDim++;
    }
    Serial.println();
  #endif
}
/****************************************************************************************************/
/* Function to load a device address from a two dimensional array Device_addresses[][] for which    */
/* is transmitted an index number. Here all addresses are identified but only the Device_Address    */
/* array is filled. The device address loaded is in Device_Address[] array.                         */
/****************************************************************************************************/
void Load_DeviceAddress(Probe_t MyProbe) {
  uint8_t *ptrTwoDim;
  uint8_t *ptrOneDim;
  uint8_t k;
  ptrOneDim = &Device_Address[0];
  switch (MyProbe) {
    case Probe1:
      ptrTwoDim = &Device_addresses[0][0];
      break;
    case Probe3:
      ptrTwoDim = &Device_addresses[2][0];
      break;
    case Probe4:
      ptrTwoDim = &Device_addresses[3][0];
      break;
  }
  for (k = 0; k < 8; k++) *(ptrOneDim++) = *(ptrTwoDim++);
}
/****************************************************************************************************/
/* Function to identify the type of the sensor provided by Maxim.                                   */
/****************************************************************************************************/
void DeviceIdentifier(uint8_t code) {
  switch (code) {
    case DS18S20MODEL:
      Serial.println(" Chip = DS18S20");          // or old DS1820
      break;
    case DS18B20MODEL:
      Serial.println(" Chip = DS18B20");
      break;
    case DS1822MODEL:
      Serial.println(" Chip = DS1822");
      break;
    default:
      Serial.println("Device does not belong to Maxim family.");
      return;
  } 
}
/****************************************************************************************************/
/* Function which displays the content of the unidimensional array of the selected address probe.   */
/****************************************************************************************************/
void AfficheAdresseCapteur(Probe_t MyProbe) {
  uint8_t *local_ptr;
  uint8_t k;
  switch (MyProbe) {
    case Probe1:
      local_ptr = &Probe1_Address[0];
      Serial.print(F("Probe1 => "));
      break;  
    case Probe3:
      local_ptr = &Probe3_Address[0];
      Serial.print(F("Probe3 => "));
      break;    
    case Probe4:
      local_ptr = &Probe4_Address[0];
      Serial.print(F("Probe4 => "));
      break;
  }
  Serial.print(F("Selected probe address: "));
  for (k = 0; k < 8; k++) {
    if (*local_ptr > 9) {
      if (*local_ptr < 16) {
        Serial.print('0');
        Serial.print(*(local_ptr++), HEX);
      } else Serial.print(*(local_ptr++), HEX);
    } else {
      Serial.print('0');
      Serial.print(*(local_ptr++), DEC);
    }
    Serial.print(' ');
  }
  Serial.println();
}
/**********************************************************************************************************************************/
/* Function to read the humidity measure from probes which are present. indice will change from 0 to 3.                           */
/* Probe2 has not to be used with the ADC converter with an I2C  address 0x49 for which the address of ADS1115 is tied to VDD.    */
/**********************************************************************************************************************************/
void AcquireHumidity(Probe_t ProbeSelected) {
  int16_t adc;
  float ddpADC;
  float HumidityRH;
  uint8_t j, k;
  float regression = RegLin;        // RegLin = -0.0078
  separateur(80, '*');
  switch (ProbeSelected) {
    case Probe1:                    // construct an ads1115 at address 0x48 (ADDR tied to GND)
      adc = ads1115_Probe1.readADC_SingleEnded(Channel_ADC);
      if (adc != -1) {
        CheckDS18B20Probe1 = true;
        Serial.println(F("Probe1 values:"));
      } else {
        CheckDS18B20Probe1 = false;
        Serial.println(F("Probe1 is not connected..."));
        return;
      }
      break;
    case Probe3:                    // construct an ads1115 at address 0x4A (ADDR tied to SDA)
      adc = ads1115_Probe3.readADC_SingleEnded(Channel_ADC);
      if (adc != -1) {
        CheckDS18B20Probe3 = true;
        Serial.println(F("Probe3 values:"));
      } else {
        CheckDS18B20Probe3 = false;
        Serial.println(F("Probe3 is not connected..."));
        return;
      }
      break;
    case Probe4:                    // construct an ads1115 at address 0x4B (ADDR tied to SCL)
      adc = ads1115_Probe4.readADC_SingleEnded(Channel_ADC);
      if (adc != -1) {
        CheckDS18B20Probe4 = true;
        Serial.println(F("Probe4 values:"));
      } else {
        CheckDS18B20Probe4 = false;
        Serial.println(F("Probe4 is not connected..."));
        return;
      }
      break;
  }
  Serial.print(F("Integer value provided by ADC: "));
  Serial.println(adc, DEC);
  ddpADC = ((float)adc/(float)valMax) * gain2_048;
  Serial.print(F("potential at the ADC1 converter input: "));
  Serial.print(ddpADC, 3);
  if (ddpADC > 2) Serial.println(" Volts");
  else Serial.println(" Volt");
  HumidityRH = (regression * (float)adc) + (float)ordonnee_origine;   // Value of humidity in Realtive Humidity unit (%)
  switch (ProbeSelected) {
    case Probe1:
      Serial.print(F("Probe1 humidity measures:"));
      break;
    case Probe3:
      Serial.print(F("Probe3 humidity measures:"));
      break;
    case Probe4:
      Serial.print(F("Probe4 humidity measures:"));
      break;
  }
    
  Serial.print(F("Relative humidity measure is "));
  Serial.print(HumidityRH);
  Serial.println(" %RH");
}
/**********************************************************************************************************************************/
/* Function to read the battery voltage. The function return a float value and the internal voltage reference is 1.2 Volt.        */
/* Problem which has to be resolved soon.                                                                                         */
/**********************************************************************************************************************************/
// https://heltec-automation-docs.readthedocs.io/en/latest/cubecell/frequently_asked_questions.html#how-to-use-the-adc-pin-for-analogread-asr6501  
void DisplayBatteryVoltage(void) {
  pinMode(ADC_CTL, OUTPUT);               // ADC_CTL = GPIO7 (see ASR_1rduino.h) (P3_3)
  digitalWrite(ADC_CTL, LOW);
  delayMicroseconds(100);
  Nbr_mV_Integer =  analogRead(ADC) * 2;  // ADC (P2_3) declared in ASR_Arduino.h
  digitalWrite(ADC_CTL, HIGH);
  BatteryVoltage = (float)Nbr_mV_Integer / 1000;
  separateur(80, '_');
  Serial.print(F("Battery voltage:  "));
  Serial.print(BatteryVoltage, 3);
  if (BatteryVoltage > 2.0) Serial.println(F("Volts"));
  else Serial.println(F("Volt"));
}
/**********************************************************************************************************************************/
/* Write function for only one byte on the OneWire bus. LSB transmitted first.                                                    */
/* for (mask = 00000001; mask>0; mask <<= 1) { //iterate through bit mask with byte mask = 1;                                     */
/* Mesaures:  Duration of slot one between two falling edge is 68 µs using parameters below.                                      */
/*            Duration of slot zero between two falling edge is 69,6 µs using parameters below.                                   */
/**********************************************************************************************************************************/
void WriteByte(uint8_t MyByte, Probe_t ProbeSelected) {
  uint8_t mask = 0x01;
  uint8_t k;
  noInterrupts();
  switch (ProbeSelected) {
    case Probe1:
      pinMode(DS18S20Pin_GPIO0, OUTPUT);
      for (k = 0; k < 8; k++) {
        digitalWrite(DS18S20Pin_GPIO0, LOW);      // falling edge to start the Slot OneWire
        if ((MyByte & mask) != 0) {               // at initial time mask = 0x01
          delayMicroseconds(4);
          digitalWrite(DS18S20Pin_GPIO0, HIGH);   // 7 µs measured
          pinMode(DS18S20Pin_GPIO0, INPUT_PULLUP);
          delayMicroseconds(20);                  // with the control of GPIO, the complete delay is 65 µs before the next falling edge
          pinMode(DS18S20Pin_GPIO0, OUTPUT);
        } else {
          delayMicroseconds(60);
          digitalWrite(DS18S20Pin_GPIO0, HIGH);
          delayMicroseconds(2);
        }
        mask <<= 1;         // mask = mask << 1;
      }
      break;
    case Probe3:
      pinMode(DS18S20Pin_GPIO1, OUTPUT);
      for (k = 0; k < 8; k++) {
        digitalWrite(DS18S20Pin_GPIO1, LOW);      // falling edge to start the Slot OneWire
        if ((MyByte & mask) != 0) {               // at initial time mask = 0x01
          delayMicroseconds(5);
          digitalWrite(DS18S20Pin_GPIO1, HIGH);
          delayMicroseconds(55);
        } else {
          delayMicroseconds(60);
          digitalWrite(DS18S20Pin_GPIO1, HIGH);
        }
        mask <<= 1;         // mask = mask << 1;
      }
      break;
    case Probe4:
      pinMode(DS18S20Pin_GPIO2, OUTPUT);
      for (k = 0; k < 8; k++) {
        digitalWrite(DS18S20Pin_GPIO2, LOW);      // falling edge to start the Slot OneWire
        if ((MyByte & mask) != 0) {               // at initial time mask = 0x01
          delayMicroseconds(5);
          digitalWrite(DS18S20Pin_GPIO2, HIGH);
          delayMicroseconds(55);
        } else {
          delayMicroseconds(60);
          digitalWrite(DS18S20Pin_GPIO2, HIGH);
        }
        mask <<= 1;         // mask = mask << 1;
      }
      break;
  }
  interrupts();
}
/**********************************************************************************************************************************/
/* Read function for only one byte on the OneWire bus.                                                                            */
/* https://en.wikipedia.org/wiki/1-Wire                                                                                           */
/* Read operations measured:  - between two falling edge for a read slot, the delay is 86,4 µs                                    */
/*                            - between 
/**********************************************************************************************************************************/
uint8_t ReadByte(Probe_t ProbeSelected) {
  uint8_t mask = 0x01;
  uint8_t k;
  uint8_t TheByte = 0;
  int BitRead;
  noInterrupts();
  switch (ProbeSelected) {
    case Probe1:
      pinMode(DS18S20Pin_GPIO0, OUTPUT);
      for (k = 0; k < 8; k++) {                       // scrolling for every bit from one byte
        digitalWrite(DS18S20Pin_GPIO0, LOW);          // falling edge to start the Slot OneWire
        pinMode(DS18S20Pin_GPIO0, INPUT_PULLUP);
        //delayMicroseconds(2);
        BitRead = digitalRead(DS18S20Pin_GPIO0);
        if (BitRead != 0) TheByte |= mask;            // https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
        delayMicroseconds(18);
        mask <<= 1;                                   // mask = mask << 1;
        pinMode(DS18S20Pin_GPIO0, OUTPUT_PULLUP);
      }
      digitalWrite(DS18S20Pin_GPIO0, HIGH);
      pinMode(DS18S20Pin_GPIO0, OUTPUT);
      break;
    case Probe3:
      pinMode(DS18S20Pin_GPIO1, OUTPUT);
      for (k = 0; k < 8; k++) {
        digitalWrite(DS18S20Pin_GPIO1, LOW);          // falling edge to start the Slot OneWire
        pinMode(DS18S20Pin_GPIO1, INPUT_PULLUP);
        //delayMicroseconds(2);
        BitRead = digitalRead(DS18S20Pin_GPIO1);
        if (BitRead != 0) TheByte |= mask;            // https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
        delayMicroseconds(18);
        mask <<= 1;                                   // mask = mask << 1;
        pinMode(DS18S20Pin_GPIO1, OUTPUT_PULLUP);
      }
      digitalWrite(DS18S20Pin_GPIO1, HIGH);
      pinMode(DS18S20Pin_GPIO1, OUTPUT);
      break;
    case Probe4:
      pinMode(DS18S20Pin_GPIO2, OUTPUT);
      for (k = 0; k < 8; k++) {
        digitalWrite(DS18S20Pin_GPIO2, LOW);          // falling edge to start the Slot OneWire
        pinMode(DS18S20Pin_GPIO2, INPUT_PULLUP);
        //delayMicroseconds(2);
        BitRead = digitalRead(DS18S20Pin_GPIO2);
        if (BitRead != 0) TheByte |= mask;            // https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html
        delayMicroseconds(18);
        mask <<= 1;                                   // mask = mask << 1;
        pinMode(DS18S20Pin_GPIO2, OUTPUT_PULLUP);
      }
      digitalWrite(DS18S20Pin_GPIO2, HIGH);
      pinMode(DS18S20Pin_GPIO2, OUTPUT);
      break;
  }
  interrupts();
  return TheByte;
}
/**********************************************************************************************************************************/
/* Function to send  a control command for ROM only (ROM commands). When we use an 0x33 command (Read ROM), the device gives the  */
/* 64-bit ROM code. The least significant byte is sent first which corresponds to the family code of the OneWire device.          */
/* https://www.maximintegrated.com/en/design/technical-documents/app-notes/1/126.html                                             */
/**********************************************************************************************************************************/
void SendRomCommand(ROM_Commands_t ROMCommandApplied, Probe_t ProbeSelected) {
  WriteByte(ROMCommandApplied, ProbeSelected);
}
/**********************************************************************************************************************************/
/* Function to apply actions in terms of control command used for ROM.                                                            */
/**********************************************************************************************************************************/
void ActionsAppliedAwaited(ROM_Commands_t ROMCommandApplied, Probe_t ProbeSelected) {
  uint8_t ByteRead;
  uint8_t k;
  switch (ProbeSelected) {
    case Probe1:
      switch (ROMCommandApplied) {
        case Search_ROM:
          break;
        case Read_ROM:
          for (k = 0; k < 8; k++) {             // least significant byte first
            ByteRead = ReadByte(Probe1);
            Probe1_Address[k] = ByteRead;       // Probe1_Address[0] contains the family code
          }
          break;
        case Match_ROM:
          break;
        case Skip_ROM:
          break;
        case Alarm_Search:
          break;
      }
      break;
    case Probe3:
      switch (ROMCommandApplied) {
        case Search_ROM:
          break;
        case Read_ROM:
          for (k = 0; k < 8; k++) {             // least significant byte first
            ByteRead = ReadByte(Probe3);
            Probe3_Address[k] = ByteRead;       // Probe1_Address[0] contains the family code
          }
          break;
        case Match_ROM:
          break;
        case Skip_ROM:
          break;
        case Alarm_Search:
          break;
      }
      break;      
    case Probe4:
      switch (ROMCommandApplied) {
        case Search_ROM:
          break;
        case Read_ROM:
          for (k = 0; k < 8; k++) {             // least significant byte first
            ByteRead = ReadByte(Probe4);
            Probe4_Address[k] = ByteRead;       // Probe1_Address[0] contains the family code
          }
          break;
        case Match_ROM:
          break;
        case Skip_ROM:
          break;
        case Alarm_Search:
          break;
      }
      break;
  }
}
/****************************************************************************************************/
/* Function which displays the content of the unidimensional array of the selected address probe.   */
/****************************************************************************************************/
void DisplayAddressProbe(Probe_t MyProbe) {
  uint8_t *local_ptr;
  uint8_t k;
  switch (MyProbe) {
    case Probe1:
      local_ptr = &Probe1_Address[0];
      Serial.print(F("Probe1 address => "));
      break;   
    case Probe3:
      local_ptr = &Probe3_Address[0];
      Serial.print(F("Probe3 address => "));
      break;    
    case Probe4:
      local_ptr = &Probe4_Address[0];
      Serial.print(F("Probe4 address => "));
      break;
  }
  for (k = 0; k < 8; k++) {
    if (*local_ptr > 9) Serial.print(*(local_ptr++), HEX);
    else {
      Serial.print('0');
      Serial.print(*(local_ptr++), DEC);
    }
    Serial.print(' ');
  }
  Serial.println();
}
/**********************************************************************************************************************************/
/* Function to reset the OneWire bus and to check the presence of each probes.                                                    */
/* disable the global flag for interruptions is necessary before configuring GPIO ports. It seems to be the rule.                 */
/**********************************************************************************************************************************/
bool ResetOneWire(Probe_t ProbeSelected) {
  int GPIOState;
  uint8_t k, tempo;
  uint8_t OutOffDelay;
  noInterrupts();
  
  switch (ProbeSelected) {
    case Probe1:
      pinMode(DS18S20Pin_GPIO0, INPUT);
      interrupts();
      do {                              // wait until the wire is high... just in case
        tempo = 0;
        for (k = 0; k < 8; k++) {       // sampling while 40 µs
          GPIOState = digitalRead(DS18S20Pin_GPIO0);
          if (GPIOState != 0) tempo++;
          delayMicroseconds(5);
        }
      } while(tempo != 8);
      noInterrupts();
      pinMode(DS18S20Pin_GPIO0, OUTPUT);
      digitalWrite(DS18S20Pin_GPIO0, LOW);
      break;
    case Probe3:
      pinMode(DS18S20Pin_GPIO1, INPUT);
      interrupts();
      do {                              // wait until the wire is high... just in case
        tempo = 0;
        for (k = 0; k < 8; k++) {       // sampling while 40 µs
          GPIOState = digitalRead(DS18S20Pin_GPIO1);
          if (GPIOState != 0) tempo++;
          delayMicroseconds(5);
        }
      } while(tempo != 8);
      noInterrupts();
      pinMode(DS18S20Pin_GPIO1, OUTPUT);
      digitalWrite(DS18S20Pin_GPIO1, LOW);
      break;
    case Probe4:
      pinMode(DS18S20Pin_GPIO2, INPUT);
      interrupts();
      do {                              // wait until the wire is high... just in case
        tempo = 0;
        for (k = 0; k < 8; k++) {       // sampling while 40 µs
          GPIOState = digitalRead(DS18S20Pin_GPIO2);
          if (GPIOState != 0) tempo++;
          delayMicroseconds(5);
        }
      } while(tempo != 8);
      noInterrupts();
      pinMode(DS18S20Pin_GPIO2, OUTPUT);
      digitalWrite(DS18S20Pin_GPIO2, LOW);
      break;
  }
  interrupts();
  delayMicroseconds(480);               // Reset pulse
  noInterrupts();                       // here the slave has the priority
  switch (ProbeSelected) {
    case Probe1:
      digitalWrite(DS18S20Pin_GPIO0, HIGH);
      pinMode(DS18S20Pin_GPIO0, INPUT_PULLUP);
      do {
        GPIOState = digitalRead(DS18S20Pin_GPIO0);    // falling edge detected
      } while(GPIOState == 1);
      OutOffDelay = 2;
      do {
        tempo = 0;
        for (k = 0; k < 12; k++) {                    // about 5 x 12 µs
          GPIOState = digitalRead(DS18S20Pin_GPIO0);
          if (GPIOState == 0) tempo++;
          delayMicroseconds(5);                       // at least the GPIO is hold low for 60 µs up to 240 µs
        }
        OutOffDelay--;
        if (OutOffDelay == 0 && tempo != 12) {
          CheckDS18B20Probe1 = false;
          return false;
        }
      } while(tempo != 12);
      do {
        GPIOState = digitalRead(DS18S20Pin_GPIO0);    // rising edge detected
      } while(GPIOState == 0);
      interrupts();                                   // the focus is returned to master
      delayMicroseconds(50);
      CheckDS18B20Probe1 = true;
      return true;
      break;
    case Probe3:
      digitalWrite(DS18S20Pin_GPIO1, HIGH);
      pinMode(DS18S20Pin_GPIO1, INPUT_PULLUP);
      do {
        GPIOState = digitalRead(DS18S20Pin_GPIO1);    // falling edge detected
      } while(GPIOState == 1);
      OutOffDelay = 2;
      do {
        tempo = 0;
        for (k = 0; k < 12; k++) {                    // about 5 x 12 µs
          GPIOState = digitalRead(DS18S20Pin_GPIO1);
          if (GPIOState == 0) tempo++;
          delayMicroseconds(5);                       // at least the GPIO is hold low for 60 µs up to 240 µs
        }
        OutOffDelay--;
        if (OutOffDelay == 0 && tempo != 12) {
          CheckDS18B20Probe3 = false;
          return false;
        }
      } while(tempo != 12);
      do {
        GPIOState = digitalRead(DS18S20Pin_GPIO1);    // rising edge detected
      } while(GPIOState == 0);
      interrupts();                                   // the focus is returned to master
      delayMicroseconds(50);
      CheckDS18B20Probe3 = true;
      return true;
      break;
    case Probe4:
      digitalWrite(DS18S20Pin_GPIO2, HIGH);
      pinMode(DS18S20Pin_GPIO2, INPUT_PULLUP);
      do {
        GPIOState = digitalRead(DS18S20Pin_GPIO2);    // falling edge detected
      } while(GPIOState == 1);
      OutOffDelay = 2;
      do {
        tempo = 0;
        for (k = 0; k < 12; k++) {                    // about 5 x 12 µs
          GPIOState = digitalRead(DS18S20Pin_GPIO2);
          if (GPIOState == 0) tempo++;
          delayMicroseconds(5);                       // at least the GPIO is hold low for 60 µs up to 240 µs
        }
        OutOffDelay--;
        if (OutOffDelay == 0 && tempo != 12) {
          CheckDS18B20Probe4 = false;
          return false;
        }
      } while(tempo != 12);
      do {
        GPIOState = digitalRead(DS18S20Pin_GPIO2);    // rising edge detected
      } while(GPIOState == 0);
      interrupts();                                   // the focus is returned to master
      delayMicroseconds(50);
      CheckDS18B20Probe4 = true;
      return true;
      break;
  }
}
/************************************************************************************************************************************/
/* Searching DS18B20 addresses at the boot of the card for each BINDER connector with a reset before sending a ROM command.         */
/* The BINDER connector number 1 is farthest the solar pannel connector.                                                            */
/* Each identified connector do not allows to invert the probes because the three I2C addresses is the only mean to identify the    */
/* location of a probe.                                                                                                             */
/* DS18S20Pin_GPIO0 <= tied to BINDER connector number 1 for probe1 and sensors_N1                                                  */
/* DS18S20Pin_GPIO1 <= tied to BINDER connector number 2 for probe3 and sensors_N3                                                  */
/* DS18S20Pin_GPIO2 <= tied to BINDER connector number 3 for probe4 and sensors_N4                                                  */
/* sensors_N1 (Ox48), sensors_N2 (Ox49), sensors_N1 (Ox4A), sensors_N1 (Ox4B)                                                       */
/************************************************************************************************************************************/
uint8_t IdentifyAddresses() {
  uint8_t Nbr = 0;
  uint8_t ADCresolution;                                        // if resolution is defined in this method

  if (ResetOneWire(Probe1) == true) {
    CheckDS18B20Probe1 == true;
    SendRomCommand(Read_ROM, Probe1);                           // just after the reset
    ActionsAppliedAwaited(Read_ROM, Probe1);                    // store the ROM code in an array
    StoreAddressRead_In_Array2dimensions(Probe1);
    Nbr++; 
    separateur(80, '_');
    Serial.println(F("Probe1 DS18B20 is present."));
    AfficheAdresseCapteur(Probe1);                              // use only the address &Probe1_Address[0];
    Serial.print(F("OneWire probe type detected: "));
    DeviceIdentifier(Probe1_Address[0]);
  } else {
    CheckDS18B20Probe1 == false;
    separateur(80, '-');
    Serial.println(F("Probe1 DS18B20 is not connected or is faulty"));
  }

  if (ResetOneWire(Probe3) == true) {
    CheckDS18B20Probe3 == true;
    SendRomCommand(Read_ROM, Probe3);
    ActionsAppliedAwaited(Read_ROM, Probe3);
    StoreAddressRead_In_Array2dimensions(Probe3);
    Nbr++; 
    separateur(80, '_');
    Serial.println(F("Probe3 DS18B20 is present."));
    AfficheAdresseCapteur(Probe3);
    Serial.print(F("OneWire probe type detected: "));
    DeviceIdentifier(Probe3_Address[0]);
  } else {
    CheckDS18B20Probe3 == false;
    separateur(80, '-');
    Serial.println(F("Probe3 DS18B20 is not connected or is faulty"));
  }

  if (ResetOneWire(Probe4) == true) {
    CheckDS18B20Probe4 == true;
    SendRomCommand(Read_ROM, Probe4);
    ActionsAppliedAwaited(Read_ROM, Probe4);
    StoreAddressRead_In_Array2dimensions(Probe4);
    Nbr++;
    separateur(80, '_');
    Serial.println(F("Probe4 DS18B20 is present."));
    AfficheAdresseCapteur(Probe4);                              // use only the address &Probe1_Address[0];
    Serial.print(F("OneWire probe type detected: "));
    DeviceIdentifier(Probe4_Address[0]);
  } else {
    CheckDS18B20Probe4 == false;
    separateur(80, '-');
    Serial.println(F("Probe4 DS18B20 is not connected or is faulty"));
  }
  return Nbr;
}
/**********************************************************************************************************************************/
/* General function to control the scratchpad registers of the device. The control command from the master imply a specific       */
/* answer. With a Convert_T command, we get the modification of the scratchpad content that is ready to be read with common       */ 
/* variables which are updated. The temperetaure values can be retrieve using tempProbex and Ds18B20Tempx.                        */
/* A reset signal is used before sending this command.                                                                            */
/**********************************************************************************************************************************/
void SendFunctionCommand(Function_Commands_t FunctionCommandApplied, Probe_t ProbeSelected) {
  uint8_t k;
  bool ProbeAnswer;
  uint16_t delms = 750;                                 // by default
  unsigned long LocalDelay;
  ProbeAnswer = ResetOneWire(ProbeSelected);
  SendRomCommand(Skip_ROM, ProbeSelected);              // page 13 because there is only one probe by GPIO
  if (ProbeAnswer == true) {
    WriteByte(FunctionCommandApplied, ProbeSelected);   // checked
  } else return;
  switch (FunctionCommandApplied) {                     // compare with requestTemperatures() : reset(), skip() and write(Convert_T)
    case Convert_T:                                     // acquire the measure
      switch (ProbeSelected) {
        case Probe1:
          LocalDelay = millis();
          while (!IsConversionComplete(Probe1) && (millis() - delms < LocalDelay));
          // Actions: reset(), select(), write(command) and read() as method readScratchPad
          ResetOneWire(Probe1);                                             // checked
          SendRomCommand(Skip_ROM, Probe1);
          WriteByte(Read_Scratchpad, Probe1);                               // command
          for (k = 0; k < 9; k++) SRAM_ScratchPad[k] = ReadByte(Probe1);    // answer of the device
          ResetOneWire(Probe1);
          Ds18B20Temp1 = TemperatureMeasure(Probe1, &SRAM_ScratchPad[0]);   // also update tempProbe1
          break;
        case Probe3:
          LocalDelay = millis();
          while (!IsConversionComplete(Probe3) && (millis() - delms < LocalDelay));
          // Actions: reset(), select(), write(command) and read() as method readScratchPad
          ResetOneWire(Probe3);
          SendRomCommand(Skip_ROM, Probe3);
          WriteByte(Read_Scratchpad, Probe3);                               // command
          for (k = 0; k < 9; k++) SRAM_ScratchPad[k] = ReadByte(Probe3);    // answer of the device
          ResetOneWire(Probe3);
          Ds18B20Temp3 = TemperatureMeasure(Probe3, &SRAM_ScratchPad[0]);   // also update tempProbe3
          break;
        case Probe4:
          LocalDelay = millis();
          while (!IsConversionComplete(Probe4) && (millis() - delms < LocalDelay));
          // Actions: reset(), select(), write(command) and read() as method readScratchPad
          ResetOneWire(Probe4);
          SendRomCommand(Skip_ROM, Probe4);
          WriteByte(Read_Scratchpad, Probe4);                               // command
          for (k = 0; k < 9; k++) SRAM_ScratchPad[k] = ReadByte(Probe4);    // answer of the device
          ResetOneWire(Probe4);
          Ds18B20Temp4 = TemperatureMeasure(Probe4, &SRAM_ScratchPad[0]);   // also update tempProbe4
          break;
      }
      break;
    case Write_Scratchpad:
      break;
    case Read_Scratchpad:
      break;
    case Copy_Scratchpad:
      break;
    case Recall_E2:
      break;
    case ReadPowerSupply:
      break;
  }
}
/************************************************************************************************************************************/
/* To check the busy state of the probe. 0 means busy state, 1 means conversion complete.                                           */
/************************************************************************************************************************************/
bool IsConversionComplete(Probe_t ProbeSelected) {
  uint8_t mask = 0x01;
  uint8_t k;
  uint8_t TheByte = 0;
  int BitRead;
  noInterrupts();
  switch (ProbeSelected) {
    case Probe1:
      pinMode(DS18S20Pin_GPIO0, OUTPUT);
      digitalWrite(DS18S20Pin_GPIO0, LOW);          // falling edge to start the Slot OneWire
      pinMode(DS18S20Pin_GPIO0, INPUT_PULLUP);
      delayMicroseconds(1);
      BitRead = digitalRead(DS18S20Pin_GPIO0);
      delayMicroseconds(14);
      break;
    case Probe3:
      pinMode(DS18S20Pin_GPIO1, OUTPUT);
      digitalWrite(DS18S20Pin_GPIO1, LOW);          // falling edge to start the Slot OneWire
      pinMode(DS18S20Pin_GPIO1, INPUT_PULLUP);
      delayMicroseconds(1);
      BitRead = digitalRead(DS18S20Pin_GPIO1);
      delayMicroseconds(14);
      break;
    case Probe4:
      pinMode(DS18S20Pin_GPIO2, OUTPUT);
      digitalWrite(DS18S20Pin_GPIO2, LOW);          // falling edge to start the Slot OneWire
      pinMode(DS18S20Pin_GPIO2, INPUT_PULLUP);
      delayMicroseconds(1);
      BitRead = digitalRead(DS18S20Pin_GPIO2);
      delayMicroseconds(14);
      break;
  }
  interrupts();
  if (BitRead != 0) return true; 
  else return false;
}
/**********************************************************************************************************************************/
/* Function to select only one device using the ROM command MATCH_ROM (0x55) and using the known address stored in an array.      */
/**********************************************************************************************************************************/
void SelectOneDevice(Probe_t ProbeSelected, uint8_t *ptr_RomCode) {
  ROM_Commands_t LocalRomCommand = Match_ROM;
  uint8_t k;
  SendRomCommand(LocalRomCommand, ProbeSelected);
  switch (ProbeSelected) {
    case Probe1:
      for (k = 0; k < 8; k++) WriteByte(*(ptr_RomCode++), Probe1);      // least significant byte first
      break;
    case Probe3:
      for (k = 0; k < 8; k++) WriteByte(*(ptr_RomCode++), Probe3);      // least significant byte first
      break;
    case Probe4:
      for (k = 0; k < 8; k++) WriteByte(*(ptr_RomCode++), Probe4);      // least significant byte first
      break;
  }
}
/**********************************************************************************************************************************/
/* Function to convert the int16_t values read in scratchpad into a float number which represents the temperature.                */
/* example from Bertrand Vandeportaele: http://homepages.laas.fr/bvandepo/wiki/doku.php?id=tp_one_wire                            */
/* Thermometer resolution is 12 by default.                                                                                       */
/**********************************************************************************************************************************/
float TemperatureMeasure(Probe_t ProbeSelected, uint8_t *ptr_scratchpad) {
  float Measure;
  switch (ProbeSelected) {
    case Probe1:
      tempProbe1 = (int16_t)((ptr_scratchpad[1]<<8) | ptr_scratchpad[0]);
      //scratch_16bitsFunc = (uint16_t)((ptr_scratchpad[1]<<8) | ptr_scratchpad[0]);
      Measure = (float)tempProbe1 * 0.0625;           // or tempProbe1 / 16
      break;
    case Probe3:
      tempProbe3 = (int16_t)((ptr_scratchpad[1]<<8) | ptr_scratchpad[0]);
      Measure = (float)tempProbe3 * 0.0625;
      break;
    case Probe4:
      tempProbe4 = (int16_t)((ptr_scratchpad[1]<<8) | ptr_scratchpad[0]);
      Measure = (float)tempProbe4 * 0.0625;
      break;
  }
  return Measure;
}
/**********************************************************************************************************************************/
/* Function to read temperature values on each probe and only used at the boot sequence of the CubeCell.                          */
/**********************************************************************************************************************************/
void AcquireTempMeasureAndDisplay(Probe_t ProbeSelected) {
  uint8_t j;
  switch (ProbeSelected) {
    case Probe1:
      if (CheckDS18B20Probe1 == true) {
        SendFunctionCommand(Convert_T, Probe1);
        ConvFloatToString(Ds18B20Temp1, 5, 2, &TabAsciiFunction[0]);
        Serial.print(F("measured temperature on probe1: "));
        for (j = 0; j < 5; j++) Serial.print((char)TabAsciiFunction[j]);
        Serial.println(F(" °C"));
      }
      break;
    case Probe3:
      if (CheckDS18B20Probe3 == true) {
        SendFunctionCommand(Convert_T, Probe3);
        ConvFloatToString(Ds18B20Temp3, 5, 2, &TabAsciiFunction[0]);
        Serial.print(F("measured temperature on probe3: "));
        for (j = 0; j < 5; j++) Serial.print((char)TabAsciiFunction[j]);
        Serial.println(F(" °C"));
      }
      break;
    case Probe4:
      if (CheckDS18B20Probe4 == true) {
        SendFunctionCommand(Convert_T, Probe4);
        ConvFloatToString(Ds18B20Temp4, 5, 2, &TabAsciiFunction[0]);
        Serial.print(F("measured temperature on probe4: "));
        for (j = 0; j < 5; j++) Serial.print((char)TabAsciiFunction[j]);
        Serial.println(F(" °C"));
      }
      break;
  }
}
/********************************************************************************************************/
/* Function to replace the equivalent method dtostrf.                                                   */
/* char *dtostrf(double val, signed char width, unsigned char prec, char *s)                            */
/* the aim is to fill the char array identified with the litteral values of a double or float number.   */
/* signed char width : number of alphanumeric values, comma included.                                   */
/* unsigned char prec : precision of the float or number of digits just after the comma.                */
/* essential link to get an example for conversion: https://www.esp8266.com/viewtopic.php?t=3592        */
/********************************************************************************************************/
void ConvFloatToString(float ConvertFloat, uint8_t Width, uint8_t NbrDecimals, char *DestArray) {
  uint8_t j, k;
  MyFloat_t LocalFloat;
  LocalFloat.value = ConvertFloat;
  uint32_t IEEE754Representation = 0;
  uint32_t IntegerResult;
  char Scratch_tab[10];       // to convert uint32_t in ASCII format
  uint8_t NbrChar;
  uint8_t CommaPosition;
  
  for (k = 4; k > 0; k--) BigEndianFormat[k - 1] = LocalFloat.byte_value[4 - k];        // big-endian representation
  for (k = 4; k > 0; k--) {
    IEEE754Representation |= LocalFloat.byte_value[k - 1];
    if (k != 1) IEEE754Representation <<= 8;
  }
  if (IEEE754Representation & Sign_Mask) {        // #define Sign_Mask 0x80000000
    *(DestArray++) = '-';
    LocalFloat.value *= -1.0;
  }
  switch (NbrDecimals) {
    case 0:
      IntegerResult = (uint32_t)LocalFloat.value;
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      for (k = 0; k < NbrChar; k++) *(DestArray++) = Scratch_tab[k];
      *DestArray = Null;
      break;
    case 1:
      IntegerResult = (uint32_t)(LocalFloat.value * 10.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 1;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      *(DestArray++) = Scratch_tab[CommaPosition];
      *DestArray = Null;
      break;
    case 2:
      IntegerResult = (uint32_t)(LocalFloat.value * 100.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 2;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 2; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 3:
      IntegerResult = (uint32_t)(LocalFloat.value * 1000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 3;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 3; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 4:
      IntegerResult = (uint32_t)(LocalFloat.value * 10000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 4;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 4; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;
    case 5:
      IntegerResult = (uint32_t)(LocalFloat.value * 100000.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 5;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 5; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;      
    default:
      IntegerResult = (uint32_t)(LocalFloat.value * 100.0);
      NbrChar = ConvertUint32ToASCIIChar(&Scratch_tab[0], IntegerResult);
      CommaPosition = NbrChar - 2;
      for (k = 0; k < CommaPosition; k++) *(DestArray++) = Scratch_tab[k];
      *(DestArray++) = '.';
      for (k = 0; k < 2; k++) *(DestArray++) = Scratch_tab[CommaPosition + k];
      *DestArray = Null;
      break;   
  }
}
/**********************************************************************************************************************************/
/* Function to convert an uint32_t (hexadecimal form) into a decimal ASCII representation to replace sprintf for long integer.    */
/* The conversion is made from low significant bit to high significant bit and the number of significant digit is returned.       */                
/**********************************************************************************************************************************/
uint8_t ConvertUint32ToASCIIChar(char *ptrTAB, uint32_t valToConvert) {   // 10 possible characters from 0 to 4,294,967,295
  char *ptrINIT;
  uint8_t m, k;
  uint8_t indice;
  uint8_t result_modulo;
  uint32_t result_division;
  ptrINIT = ptrTAB;
  for (m = 0; m < 10; m++) *(ptrTAB++) = Null;      // initialisation of the array
  indice = 9;                                       // the low significant digit in the char array (unity)
  ptrTAB = ptrINIT;
  ptrTAB += indice * sizeof(char);                  // to fix the low digit
  do {
    result_modulo = (uint8_t)(valToConvert % 0x0A);
    *(ptrTAB--) = (char)(result_modulo + 0x30);     // ASCII char to display
    indice--;
    result_division = (valToConvert - result_modulo) / 0x0A;
    valToConvert = result_division;                 // new value for which we have to define the modulo
  } while (result_division > 15);                   // if result is between 0 and 15, we can identify all characters to be displayed
  if (result_division >= 1 && result_division <= 9) *ptrTAB = (char)(0x30 + result_division);
  if (result_division >= 10 && result_division <= 15) {
    scratch_8bitsFunc = result_division - 0x0A;
    *(ptrTAB--) = (char)(0x30 + scratch_8bitsFunc);
    indice--;
    *ptrTAB = (char)0x31;
  }
  ptrTAB = ptrINIT;                                 // first position in the array
  ptrINIT += indice * sizeof(char);                 // to retrieve the last position of the most significant digit
  for (k = 0; k < 10 - indice; k++) *(ptrTAB++) = *(ptrINIT++);   // to retrieve an array starting with the fisrt position [0]
  for (k = 10 - indice; k < 10; k++) *(ptrTAB++) = 0x20;          // 10 is the dimension of the array Scratch_tab[]
  return 10 - indice;
}
/**********************************************************************************************************************************/
/* Reset a probe among 3. The control command is 'reset_'<d>, d: from 1 to 4.                                                     */              
/**********************************************************************************************************************************/
void OneWireReset(String Cde_received) {
  Probe_t ProbeSelected;
  bool Answer;
  uint8_t Nbr;
  Cde_received = Cde_received.substring(6);
  Cde_received.toCharArray(TabAsciiFunction, Cde_received.length() + 1);
  Nbr = (uint8_t)TabAsciiFunction[0] - 0x30;
  ProbeSelected = (Probe_t)Nbr;
  Answer = ResetOneWire(ProbeSelected);
  switch (ProbeSelected) {
    case Probe1:
      if (Answer) Serial.println(F("The probe1 has received a reset command"));
      else Serial.println(F("The probe1 DS18B20 is not connected or is faulty"));
      break;
    case Probe2:
      Serial.println(F("The probe2 has not to be used"));
      break;
    case Probe3:
      if (Answer) Serial.println(F("The probe3 has received a reset command"));
      else Serial.println(F("The probe3 DS18B20 is not connected or is faulty"));
      break;
    case Probe4:
      if (Answer) Serial.println(F("The probe4 has received a reset command"));
      else Serial.println(F("The probe4 DS18B20 is not connected or is faulty"));
      break;
  }
}



/* ######################################################################################################## */
// END of file
