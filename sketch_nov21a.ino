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

int scale = 5;
float threshold = 3;
int shakes = 0;
int time = millis();

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

  //ble.verbose(false);

  Serial.println(F("Adding the Handshake Service definition "));
  int success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID128=00-77-13-12-11-00-00-00-00-00-AB-BA-0F-A1-AF-E1"), &hsServiceId);
  if (!success) {
    error(F("Could not add Handshake service"));
  }

  Serial.println(F("Adding the Handshake Data characteristic "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=00-68-42-01-14-88-59-77-42-42-AB-BA-0F-A1-AF-E1,PROPERTIES=0x12,MIN_LEN=1,VALUE=0"), &hsDataCharId);
  if (!success) {
    error(F("Could not add Handshake characteristic"));
  }

  Serial.println(F("Adding Handshake Service UUID to the advertising payload "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-03-02-12-13") );

  Serial.println(F("Performing a SW reset (service changes require a reset) "));
  ble.reset();
}

void loop()
{  
  // Get raw accelerometer data for each axis
  int rawX = analogRead(A0);
  //int rawY = analogRead(A1);
  //int rawZ = analogRead(A2);
  
  // Scale accelerometer ADC readings into common units
  // Scale map depends on if using a 5V or 3.3V microcontroller
  float scaledX, scaledY, scaledZ; // Scaled values for each axis
  scaledX = mapf(rawX, 0, 675, -scale, scale);
  //scaledY = mapf(rawY, 0, 675, -scale, scale);
  //scaledZ = mapf(rawZ, 0, 675, -scale, scale);

  if(abs(scaledX) >= threshold && millis() - time < 1000) {
    Serial.write("H++");
    shakes++;
  }

  if(shakes > 6){
    Serial.write("HANDHSAKE");
    ble.print( F("AT+GATTCHAR=") );
    ble.print( hsDataCharId );
    ble.print( ",1" );

    if ( !ble.waitForOK() ){
      Serial.println(F("Failed to get response!"));
    }
  
    shakes = 0;
  }

  if(millis() - time >= 1000){
    shakes = 0;
    time = millis();
   }
  
  delay(20);
}

// Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

