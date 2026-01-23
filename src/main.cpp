#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include <modules/SX126x/patches/SX126x_patch_scan.h>

#define RECEIVER

#if defined(ESP8266) || defined(ESP32)
  #define LORA_MISO 8
  #define LORA_SCK 7
  #define LORA_MOSI 9
  #define LORA_CS 41
  #define LORA_DIO2 38
  #define LORA_DIO1 39
  #define LORA_RESET 42
  #define LORA_BUSY 40
#elif defined(PICO_RP2040)
  #define LORA_MISO 12
  #define LORA_SCK 10
  #define LORA_MOSI 11
  #define LORA_CS 3
  #define LORA_DIO2 21
  #define LORA_DIO1 20
  #define LORA_RESET 15
  #define LORA_BUSY 2
#endif

SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_BUSY);

volatile bool operationDone = false;
bool transmitFlag = false;
int transmissionState = RADIOLIB_ERR_NONE;

#if defined(ESP8266) || defined(ESP32)
ICACHE_RAM_ATTR
#endif
void setFlag(void) {operationDone = true;}

bool validateParameters(float frequency, float bandwidth, uint8_t spreadingFactor, uint8_t codingRate, uint8_t syncWord, float outputPower, uint16_t preambleLength) {
  if (frequency < 150.0 || frequency > 960.0) {Serial.println(F("Error: Frequency must be between 150.0 MHz and 960.0 MHz.")); return false;}
  if (bandwidth != 7.8 && bandwidth != 10.4 && bandwidth != 15.6 && bandwidth != 20.8 && bandwidth != 31.25 && bandwidth != 41.7 && bandwidth != 62.5 && bandwidth != 125.0 && bandwidth != 250.0 && bandwidth != 500.0) {Serial.println(F("Error: Invalid bandwidth value.")); return false;}
  if (spreadingFactor < 6 || spreadingFactor > 12) {Serial.println(F("Error: Spreading factor must be between 6 and 12.")); return false;}
  if (codingRate < 5 || codingRate > 8) {Serial.println(F("Error: Coding rate must be between 5 and 8.")); return false;}
  if (outputPower < -17.0 || outputPower > 22.0) {Serial.println(F("Error: Output power must be between -17 dBm and 22 dBm.")); return false;}
  if (preambleLength < 6 || preambleLength > 65535) {Serial.println(F("Error: Preamble length must be between 6 and 65535 symbols.")); return false;}
  if (syncWord > 0xFF) {Serial.println(F("Error: Sync word must be a valid 1-byte value.")); return false;}
  return true;
}

void initRadio() {
  float frequency = 869.525; //915.0
  float bandwidth = 125;   //125.0;
  uint8_t spreadingFactor = 7;
  uint8_t codingRate = 5;  // 5;
  uint8_t syncWord = 0x12;    //0x34(44) for public
  float outputPower = 14;   //17;
  uint16_t preambleLength = 8;  //10;

  if (!validateParameters(frequency, bandwidth, spreadingFactor, codingRate, syncWord, outputPower, preambleLength)) {Serial.println(F("Error: Invalid parameters. Stopping.")); while(true){delay(10);}}

  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin(frequency, bandwidth, spreadingFactor, codingRate, syncWord, outputPower, preambleLength, 1.6, false);

  if (state == RADIOLIB_ERR_NONE) {Serial.println(F("success!")); radio.setDio1Action(setFlag);}
  else {Serial.print(F("failed, code ")); Serial.println(state); while(true){delay(10);}}
}

void sendPacket(const char* message) {
  Serial.print(F("[SX1262] Sending packet ... "));
  Serial.print(message);
  Serial.print(F(" "));
  transmissionState = radio.startTransmit(message);
  transmitFlag = true;
}

void receivePacket() {
  String str;
  int state = radio.readData(str);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1262] Received packet!"));
    Serial.print(F("[SX1262] Data: "));
    Serial.println(str);
    Serial.print(F("[SX1262] RSSI: "));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));
    Serial.print(F("[SX1262] SNR: "));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));
  }
  else {
    Serial.print(F("readData failed, code "));
    Serial.println(state);
  }
}

void setup() {
  #if defined(ESP8266) || defined(ESP32)
  #elif defined(PICO_RP2040)
    SPI.setRX(LORA_MISO);
    SPI.setTX(LORA_MOSI);
    SPI.setSCK(LORA_SCK);
  #endif

  Serial.begin(9600);
  delay(10000);
  Serial.println("SETUP");
  initRadio();
  #ifdef RECEIVER
    radio.startReceive();
  #else
    sendPacket("test");
  #endif
}

void loop() {
  #ifdef RECEIVER
  if (operationDone) {
    operationDone = false;
    if (transmitFlag) {
      if (transmissionState == RADIOLIB_ERR_NONE) {Serial.println(F("transmission finished!"));}
      else {Serial.print(F("failed, code ")); Serial.println(transmissionState);}

      radio.startReceive();
      transmitFlag = false;
    }
    else {
      receivePacket();
      delay(3000);
    }
  }
  #else
    char msg[20];
    
    int uptime = millis() / 1000;

    sprintf(msg, "uptime: %d", uptime);
    sendPacket(msg);
    delay(3000);
  #endif
}