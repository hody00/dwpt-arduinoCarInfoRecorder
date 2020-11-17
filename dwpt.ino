//#include <TinyGPS.h>
#include "TinyGPS.h"
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

/* 
 *  variables explained by order:
 *  chipSelect     - pin 8 used for chip select on SD card SPI comm.
 *  labels_gps     - labels for fields in CSV file.
 *  gpsLatitude..  - variables to hold the GPS information. 
 *  isStopped      - indicates whether the car stopped.
 *  timeCount      - counts how much time the car doesn't move.
 *  timeStopped    - the exact time in seconds that the car has stopped.
 *  writingTime    - how much time we want to write gps data while the car doesn't move.
 *  gpsFile        -  used as file name.
 *  Neo7M neo      - declare the gps component.
 *  R_shant        - shant resistance.
 *  A_v            - amplifier gain.
 *  RXPin, TXPin   - GPS device hooked up on pins 0(rx) and 1(tx).
 *  GPSBaud        - the baud of our serial.
 */
const int chipSelect = 8; 
String labels_gps = "Latitude (deg), Longitude (deg), Altitude (m), Speed (km/h), CMG (deg), Date, Time, ShuntVoltage (V), Current (A), BattaryVoltage (V), Power (Watt)";
String gpsLatitude, gpsLongitude, gpsAltitude, gpsSpeed, CMG, currentDate, currentTime;
bool isStopped;
int timeCount;
double timeStopped;
int writingTime;
String gpsFile = "DATA"; 
TinyGPSPlus neo;
float R_shant = 400*(10^-6);
float A_v = 200;
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600;
// The serial connection to the GPS device
SoftwareSerial sCOMM(RXPin, TXPin);


/*
 * the setup part:
 * 1. opens a csv file on the sd card and writes the lables into the file.
 * 2. set default values in case something went wrong.
 */
void setup() {

  if (!SD.begin(chipSelect)) {
    // the speed of communication with the serial is 9600 bits per second (baud)
    Serial.begin(GPSBaud);
    Serial.print("error opening SD card"); 
    Serial.end();  
    return;
  }
  else {
      Serial.begin(GPSBaud);
      Serial.println("card initializing...");
      Serial.end();
       /*
        * check if a file named "DATA.0" exist. 
        * if it does, then up the count to see if "DATA.1" exist and so on.
        * when the loop is over we'll have a new gpsFile name thats not in use.
        * (isFile - tracks when we found a file name we can use)
        */
       bool isFile = false;
       int count = 0;
       while(!isFile){
         if(SD.exists(gpsFile + ".csv")) {
          count++;
          gpsFile = "DATA" + (String)count;
          }
         else isFile = true; 
       } 

    // create / open a csv file to write labels
    File myFile = SD.open((gpsFile + ".csv"), FILE_WRITE); 
    if (myFile) {
       myFile.println(labels_gps);
       myFile.close();
       Serial.begin(GPSBaud);
       Serial.println("a new file named " + gpsFile + " has opened"); 
       Serial.end();
       }
    //if file doesn't open - alert user
    else { 
    Serial.begin(GPSBaud);
    Serial.println("error opening file: " + gpsFile + ".csv!!"); 
    Serial.end();
    }
   
    // initialize default values 
    gpsLatitude = "N/A";
    gpsLongitude = "N/A";
    gpsAltitude = "N/A";
    gpsSpeed = "N/A";
    CMG = "N/A";
    currentDate = "00/00/0000";
    currentTime = "00:00:00:00";
    isStopped = false;
    timeCount = 0;
    timeStopped = 0;
    writingTime = 5;
  }
}


void loop() {
   // read arduino voltage data and calculate the original voltages.
  int sensorValue_shuntVoltage = analogRead(A0);
  int sensorValue_battaryVoltage = analogRead(A1);
  // arduino can get voltage up to 5.0V and converts it to bits up to 10 bits (2^10=1024), we convert back from bits to voltage
  float shuntVoltage = sensorValue_shuntVoltage * (5.0 / 1023.0);
  float battaryVoltage = sensorValue_battaryVoltage * (5.0/1023.0);
  // float current = shunt_voltage/R_shant;
  float current = shuntVoltage*300/5;
  float battaryVoltage1 = battaryVoltage*72/5;
  float power= (battaryVoltage1*current);

  // debug
  Serial.begin(GPSBaud);
  Serial.print(F("IN voltage from R-Shunt:        "));
  Serial.println(shuntVoltage,3);
  Serial.print(F("The current is:                 "));
  Serial.println(current,3);
  Serial.print(F("The Power is:                   "));
  Serial.println(power,3);
  Serial.print(F("The battary_voltage in Arduino: "));
  Serial.println(battaryVoltage1,6);
  Serial.end();
  
  // if this is true - we got GPS data so set our gps values
  while (sCOMM.available() > 0) {
    if (neo.encode(sCOMM.read())) {
      setGPSdata();
      // debug
      Serial.begin(GPSBaud);
      Serial.print(F("lat:  "));    Serial.println(gpsLatitude);
      Serial.print(F("lng:  "));    Serial.println(gpsLongitude);
      Serial.print(F("alt:  "));    Serial.println(gpsAltitude);
      Serial.print(F("kmph: "));    Serial.println(gpsSpeed);
      Serial.print(F("cmg:  "));    Serial.println(CMG);
      Serial.print(F("date: "));    Serial.println(currentDate);
      Serial.print(F("time: "));    Serial.println(currentTime);
    } else {
      Serial.begin(GPSBaud);
      Serial.println(F("failed to get data..."));
      Serial.end();
    }
  }
  
  /*
   * opens our DATA.# file for writing gps data by the pre-select labels order:
   * "Latitude, Longitude, Altitude, Speed, CMG, Date, Time, ShuntVoltage, Current, BattaryVoltage, Power".
   */
   File dataFile_g = SD.open((gpsFile + ".csv"), FILE_WRITE);
   Serial.begin(GPSBaud);
   Serial.println(gpsFile);
   Serial.end();
   // if the car is stopping for more than 5 seconds, then there is no need to write the gps data (because they haven't changed)
   if (timeCount < writingTime) {
    dataFile_g.print(gpsLatitude + ",");
    dataFile_g.print(gpsLongitude + ",");
    dataFile_g.print(gpsAltitude + ",");
    dataFile_g.print(gpsSpeed + ",");
    dataFile_g.print(CMG + ",");
    dataFile_g.print(currentDate + ",");
    dataFile_g.print(currentTime + ",");
    dataFile_g.print(String(shuntVoltage) + ",");
    dataFile_g.print(String(current) + ",");
    dataFile_g.print(String(battaryVoltage1) + ",");
    dataFile_g.println(String(power) + ",");
   }
}

/*
 * set the data received from the GPS chip: longitude, latitude, date (foramt: 00/00/0000) and time (format: 00:00:00).
 */
void setGPSdata() {
  if (neo.location.isValid()) {
    gpsLatitude = String(neo.location.lat(), 6);
    gpsLongitude = String(neo.location.lng(), 6);
  } else {Serial.println("location invalid");}
  gpsAltitude = String(neo.altitude.meters(), 6);
  gpsSpeed = String(neo.speed.kmph(), 6);
  CMG = String(neo.course.deg()), 6;
  bool isDateChanged = false;
  if (neo.time.isValid()) {
    String hours, minutes, seconds, centiseconds;
    hours = String(neo.time.hour() + 2); // israel time is UTC+2:00
    minutes = String(neo.time.minute());
    seconds = String(neo.time.second());
    if ((neo.time.hour() + 2)  > 24) {
      hours = "0" + String((neo.time.hour() + 2) - 24);
      isDateChanged = true;
    } else if ((neo.time.hour() + 2)  < 10) {
      hours = "0" + String(neo.time.hour() + 2);
    } else if ((neo.time.hour() + 2)  == 24) {
      hours = "00";
    }
    if (neo.time.minute() < 10) {
      minutes = "0" + String(neo.time.minute());
    }
    if (neo.time.second() < 10) {
      seconds = "0" + String(neo.time.second());
    }
    if (neo.time.centisecond() < 10) {
      centiseconds = "0" + String(neo.time.centisecond());
    }
    currentTime = hours + ":" + minutes + ":" + seconds + ":" + centiseconds;
  } else {Serial.println("time invalid");}
  if (neo.date.isValid()) {
    currentDate = String(neo.date.day()) + "/" + String(neo.date.month()) + "/" + String(neo.date.year());
    if (isDateChanged) {
      currentDate = String(neo.date.day() + 1) + "/" + String(neo.date.month()) + "/" + String(neo.date.year());
      }
  } else {Serial.println("date invalid");}
  // checks whether the car stopped and if it does, starts counting the time of the stopping
  if (gpsSpeed == 0) {
    isStopped = true;
    // count how much time the car doesn't move
    if (timeStopped == 0 ) {
      timeStopped = neo.time.second();
    } else {
      timeCount = neo.time.second() - timeStopped;
      }
    } else {
      // initialize the vars because the car is moving
      isStopped = false;
      timeStopped = 0;
      timeCount = 0;
      }
}
