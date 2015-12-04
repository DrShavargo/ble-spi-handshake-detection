/*********************************************************************
 Author: DrShavargo (Georges-Antoine Assi)
 Source: https://github.com/DrShavargo/
 
 This is a handshake detection and reporting example built for 
 Adafruit nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "BluefruitConfig.h"

// Sets whether BLE module factory resets when powered
#define FACTORYRESET_ENABLE      1

// Create the bluefruit object, either software serial...uncomment these lines
//SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);
//
//Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
//                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);


// ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line
//Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

// ...or hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// ...or software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// The service information
int32_t hsServiceId;
int32_t hsDataCharId;

// Handshake detection thresholds for the ADXL336
float thresholdX = 135;
float thresholdY = 270; //or more
float thresholdZ = 220;

// Partial handshake counter
int shakes = 0;

// Maximum time a handshake should last (in milliseconds)
int maxShakeTime = 1000;

// Number of partial handshakes withing threshold needed for true handshake detected
int neededShakeCount = 6;

// Max raw accelerometer values for the board
int maxRawAccValue = 675;
unsigned long time = millis();

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup()
{
  while (!Serial); // required for Flora & Micro
  delay(500);
  
  Serial.begin(115200);
  Serial.println(F("Handshake Detection and Reporting"));
  Serial.println(F("---------------------------------"));
  
  // Initialise the module
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) ){ 
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?")); 
  }
  Serial.println( F("OK!") );

  // Perform a factory reset to make sure everything is in a known state
  if ( FACTORYRESET_ENABLE ){
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  // Disable command echo from Bluefruit
  ble.echo(false);

  // Print Bluefruit information
  Serial.println("Requesting Bluefruit info:");
  ble.info();

  // Prints out debugging information when truthy
  ble.verbose(true);

  // Change the device name to make it easier to find
  Serial.println(F("Setting device name to 'SmartBand A': "));
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=SmartBand A")) ) {
    error(F("Could not set device name?"));
  }

  // Add the Handshake Service definition
  // Service ID should be 1
  Serial.println(F("Adding the Handshake Service definition "));
  int success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID128=00-77-13-12-11-00-00-00-00-00-AB-BA-0F-A1-AF-E1"), &hsServiceId);
  if (!success) {
    error(F("Could not add Handshake service"));
  }

  // Add the Handshake characteristic
  // Chars ID for Measurement should be 1
  Serial.println(F("Adding the Handshake Data characteristic "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID128=00-68-42-01-14-88-59-77-42-42-AB-BA-0F-A1-AF-E1,PROPERTIES=0x12,MIN_LEN=1,VALUE=0"), &hsDataCharId);
  if (!success) {
    error(F("Could not add Handshake characteristic"));
  }

  // Add the Handshake Service to the advertising data (needed for Nordic apps to detect the service)
  Serial.println(F("Adding Handshake Service UUID to the advertising payload "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-03-02-12-13") );

  // Reset the device for the new service setting changes to take effect
  Serial.println(F("Performing a SW reset (service changes require a reset) "));
  ble.reset();

  Serial.println();
}

void loop()
{
  
  // Get raw accelerometer data for each axis
  int rawX = analogRead(A3);
  int rawY = analogRead(A4);
  int rawZ = analogRead(A5);

  // Check if each datum is within the threshold, and increment partial shake count if true
  if(within(rawX, thresholdX)
    && withinI(rawY, thresholdY)
    && withinI(rawZ, thresholdZ)
    && millis() - time < 1000) {
    shakes++;
  }

  // Write 1 to the Handshake characteristic and reset the partial shake count
  if(shakes > neededShakeCount){  
    ble.print( F("AT+GATTCHAR=") );
    ble.print( hsDataCharId );
    ble.println( ",1" );
    ble.waitForOK();
    Serial.println("HANDSHAKE DETECTED!");
    shakes = 0;
  }

  // Reset the partial shake count after maxShakeTime
  if(millis() - time >= maxShakeTime){
    shakes = 0;
    time = millis();
   }
  
  delay(20);
}

/* 
 *  With the ADXL336, a read of 0 is -5g, and a read of 675 is 5g,
 *  0g is located at 337.5
 *  This will vary from board to board, so set maxRawAccValue accordingly
 */

/*
 * Check if x is pulling higher Gs then t
 *  |XXXXXXX|   OR   |XXXXXXX|
 *  0       t               675
 */
bool within(float x, float t){
  return ((x < t) || (maxRawAccValue-x < t));
}

/*
 * Check if x is pulling lower Gs then t
 *  |       |XXXXXXX|       |
 *  0       t              675
 */
bool withinI(float x, float t){
  return ((x > t) && (maxRawAccValue-x > t));
}

