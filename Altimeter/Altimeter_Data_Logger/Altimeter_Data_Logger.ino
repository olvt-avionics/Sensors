//for communication with sd card
#include <SPI.h>
#include <SD.h>

//Find Sd Card
const int chipSelect = BUILTIN_SDCARD;

//setup index for values 
int Index = 0;

#include<MS5607.h>

MS5607 P_Sens;

void setup(void){
  Serial.begin(9600);
  if(!P_Sens.begin()){
    Serial.println("Error in Communicating with sensor, check your connections!");
  }else{
    Serial.println("MS5607 initialization successful!");
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

//Sensor setup settings
File setupFile = SD.open("Sensor_Settings.txt", FILE_WRITE);
//Temperature Sensor Info
    setupFile.println(" ");
    setupFile.println("New Recording Settings");    
    setupFile.println("Altimeter MS-5607");
    setupFile.println("Output: C, mbar, meters");
    setupFile.close();

// Notify of new recording on sd card 
File dataFile = SD.open("datalog.txt", FILE_WRITE);
//Temperature Sensor Info
    dataFile.println(" ");
    dataFile.println("Start New Recording");    
    dataFile.close();
    
  
}
float P_val,T_val,H_val;
void loop(void){
  if(P_Sens.readDigitalValue()){
    T_val = P_Sens.getTemperature();
    P_val = P_Sens.getPressure();
    H_val = P_Sens.getAltitude();
  }else{
    Serial.println("Error in reading digital value in sensor!");
  }

// open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
     if (dataFile) {
      dataFile.println(String(Index)+ "," + String(T_val) + "," + String(P_val) + "," + String(H_val) + ",");
      Index += 1;
      dataFile.close();
     }



  Serial.print("Temperature :  ");
  Serial.print(T_val);
  Serial.println(" C");
  Serial.print("Pressure    :  ");
  Serial.print(P_val);
  Serial.println(" mBar");
  Serial.print("Altitude    :  ");
  Serial.print(H_val);
  Serial.println(" meter");
  Serial.println(" ");

  delay(1000);
}
