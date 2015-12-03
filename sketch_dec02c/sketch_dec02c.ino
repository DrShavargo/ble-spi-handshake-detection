#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"

#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE      1

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

int32_t hsServiceId;
int32_t hsDataCharId;

float thresholdX = 135;
float thresholdY = 270; //or more
float thresholdZ = 220;
int shakes = 0;
unsigned long time = millis();

void setup()
{
  Serial.begin(115200);
  Serial.print(F("Initialising the module: "));

  if ( !ble.begin(VERBOSE_MODE) ){ error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?")); }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE ){
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  ble.info();

  // MAKE TRUE IF DEBUGGING
  ble.verbose(false);

  Serial.println(F("Setting device name to 'SmartBand B': "));
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=SmartBand B")) ) {
    error(F("Could not set device name?"));
  }
  
  Serial.println(F("Performing a SW reset (service changes require a reset) "));
  ble.reset();
}

void loop()
{
  
}

