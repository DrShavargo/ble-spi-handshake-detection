# Handshake Detection with BluetoothLE Reporting
Detects handshakes and writes to GATT service characteristic on detection. Designed around Adafruit's BlueFruit LE SPI Friend and the ADXL337 accelerometer.

You will need the [Arduino library for nRF51822-based BLE modules](https://github.com/adafruit/Adafruit_BluefruitLE_nRF51), which can be found on Adafruit's Github. Download the .zip file and import it into the Arduino desktop app.

## Recommended Hardware

[Adafruit Bluefruit LE SPI Friend - Bluetooth Low Energy (BLE)](https://www.adafruit.com/products/2633)  
The UART version is also supported, though the appropriate lines must be uncommented.

[SparkFun Triple Axis Accelerometer Breakout - ADXL337](https://www.sparkfun.com/products/12786)  
Most SparkFun, and many third-party, **analog** accelerometers should work. The thresholds and maximum raw accelerometer values should be modified according to the board's specifications.

[Arduino Uno](https://www.arduino.cc/en/Main/ArduinoBoardUno)  
I used the Uno since it was the most conveniant choice, but most board should work.
