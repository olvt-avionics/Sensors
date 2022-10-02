/*
  SD card datalogger

  This example shows how to log data from three analog sensors
  to an SD card using the SD library.

  The circuit:
   analog sensors on analog ins 0, 1, and 2
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

*/

//for communication with sd card
#include <SPI.h>
#include <SD.h>

//headers for sensor
#include <Wire.h>
#include "Adafruit_MCP9808.h"

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

const int chipSelect = BUILTIN_SDCARD;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
    Serial.println("MCP9808 Logging"); {
    ; // wait for serial port to connect. Needed for native USB port only
  }

tempsensor.setResolution(3);

// sets the resolution mode of reading, the modes are defined in the table bellow:
// Mode Resolution SampleTime
//  0    0.5°C       30 ms
//  1    0.25°C      65 ms
//  2    0.125°C     130 ms
//  3    0.0625°C    250 ms



  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x19) for example, also can be left in blank for default address use
  // Also there is a table with all addres possible for this sensor, you can connect multiple sensors
  // to the same i2c bus, just configure each sensor with a different address and define multiple objects for that
  //  A2 A1 A0 address
  //  0  0  0   0x18  this is the default address
  //  0  0  1   0x19
  //  0  1  0   0x1A
  //  0  1  1   0x1B
  //  1  0  0   0x1C
  //  1  0  1   0x1D
  //  1  1  0   0x1E
  //  1  1  1   0x1F


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");


if (!tempsensor.begin(0x18)) {
  Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
  while (1);
}
Serial.println("Found MCP9808!");

}


void loop() {

//  Serial.println("wake up MCP9808.... "); // wake up MCP9808 - power consumption ~200 mikro Ampere
  tempsensor.wake();   // wake up, ready to read!

  // Read and print out the temperature, also shows the resolution mode used for reading.
  //Serial.print("Resolution in mode: ");
  //Serial.println (tempsensor.getResolution());
  float Temp_c = tempsensor.readTempC();
  float Temp_f = tempsensor.readTempF();
  //Serial.print("Temp: ");
  //Serial.print(Temp_c, 4); Serial.print("*C\t");
  //Serial.print(Temp_f, 4); Serial.println("*F.");

  delay(2000);
  //Serial.println("Shutdown MCP9808.... ");
  tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere, stops temperature sampling
  Serial.println("");
  delay(200);


  // make a string for assembling the data to log:
 // String dataString = "";

  // read three sensors and append to the string:
  //dataString += ",";
  
  //for (int analogPin = 0; analogPin < 3; analogPin++) {
    //int sensor = analogRead(analogPin);
    //dataString += String(sensor);
    //if (analogPin < 2) {
      //dataString += ",";
    //}
  //}

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(Temp_c);
    dataFile.close();
    // print to the serial port too:
    Serial.print("Temp: ");
    Serial.print(Temp_c, 4);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
