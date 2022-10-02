
/**************************************************************************/
/*!
This is a demo for the Adafruit ADT7410 breakout
----> http://www.adafruit.com/products/4089
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!
*/
/**************************************************************************/
//for communication with sd card
#include <SPI.h>
#include <SD.h>

//Find Sd Card
const int chipSelect = BUILTIN_SDCARD;

//for adt-7410
#include <Wire.h>
#include "Adafruit_ADT7410.h"

// Create the ADT7410 temperature sensor object
Adafruit_ADT7410 tempsensor = Adafruit_ADT7410();

void setup() {
  Serial.begin(115200);
  Serial.println("ADT7410 demo");
  
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x49) for example
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find ADT7410!");
    while (1);
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do a nything more:
    while (1);
  }
  Serial.println("card initialized.");

//Sensor setup settings
File dataFile = SD.open("Sensor_Settings.txt", FILE_WRITE);
//Temperature Sensor Info    
    dataFile.println("Temperature Sensor ADT-7410");
    dataFile.println("Currently Reporting in fahrenheit");
    dataFile.close();

  // sensor takes 250 ms to get first readings
  delay(250);

  
}

void loop() {
  // Read and print out the temperature, then convert to *F
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  Serial.print("Temp: "); 
  //Serial.print(c); Serial.print("*C\t"); 
  Serial.print(f); Serial.println("*F");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
     if (dataFile) {
      dataFile.println(String(f) + ",");
      dataFile.close();
     }

  
  delay(500);
}
