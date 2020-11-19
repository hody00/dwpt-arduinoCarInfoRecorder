#include "TinyGPS.h"
#include "UsbFat.h"
#include "masstorage.h"
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Limits.h>

/* 
 *  variables explained by order:
 *  chipSelect     - pin 8 used for chip select on SD card SPI comm.
 *  labels_gps     - labels for fields in CSV file.
 *  gpsLatitude..  - variables to hold the GPS information. 
 *  isStopped      - indicates whether the car stopped.
 *  timeCount      - counts how much time the car doesn't move.
 *  timeStopped    - the exact time in seconds that the car has stopped.
 *  writingTime    - how much time we want to write gps data while the car doesn't move.
 *  gpsFileName    -  used as file name.
 *  fileType       - all data will be saved in .csv type of files.
 *  pathName       - full path name, example: "DATA1.csv".
 *  Neo7M neo      - declare the gps component.
 *  R_shant        - shant resistance.
 *  A_v            - amplifier gain.
 *  RXPin, TXPin   - GPS device hooked up on pins 0(rx) and 1(tx).
 *  GPSBaud        - the baud of our serial.
 *  USB usb        - USB host object.
 *  bulk(&usb)     - don't know why, but it must be here (this piece of code was taken from https://github.com/greiman/UsbFat.git).
 *  key(&bulk)     - File system.
 *  File gpsFile   - the file that will be used to write and save gps data.
 */
const int chipSelect = 8; 
String labels_gps = "Latitude (deg), Longitude (deg), Altitude (m), Speed (km/h), CMG (deg), Date, Time, ShuntVoltage (V), Current (A), BattaryVoltage (V), Power (Watt)";
String gpsLatitude, gpsLongitude, gpsAltitude, gpsSpeed, CMG, currentDate, currentTime;
bool isStopped;
int timeCount;
double timeStopped;
int writingTime;
String gpsFileName = "DATA";
String fileType = ".csv";
const char* pathName;
TinyGPSPlus neo;
float R_shant = 400*(10^-6);
float A_v = 200;
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 115200;
// The serial connection to the GPS device
SoftwareSerial sCOMM(RXPin, TXPin);
USB usb;
BulkOnly bulk(&usb);
UsbFat key(&bulk);
File gpsFile;

/*
 * the setup part:
 * 1. opens a csv file on the USB thumb drive and writes the lables into the file.
 * 2. set default values in case something will go wrong.
 */
void setup() {

  /*
   * the next two "if" segments were taken from UsbFat library's example (UsbFatDemo).
   * first if segment: initialize the USB bus.
   * second if segment: initialize the USB key or USB hard drive.
   */
  if (!initUSB(&usb)) {
    Serial.println(F("initUSB failed"));
    return;    
  }
  if (!key.begin()) {
    Serial.println(F("key.begin failed"));
    return;
  }

  Serial.begin(GPSBaud);
  Serial.println("USB initializing...");
  Serial.end();
  
  /*
   * check if a file named "DATA1.csv" exist. if it does, then up the count to see if "DATA2.csv" exist, and so on..
   * when we find a new file name thats not in use, we create and open this file. than, we write the gps labels into it.
   * (isFile - tracks when we found a file name that we can use)
   */
    bool isFile = false;
    int count = 1;
    while (!isFile) {
      String strName = gpsFileName + (String)count + ".csv";
       // convert from String type to const char* type, in order to use "open" function later.
      pathName = strName.c_str();
      // if a file named "DATA1.csv" is exist then check if "DATA2.csv" is exist, and so on..
      if (key.exists(pathName)) {
        Serial.begin(GPSBaud);
        Serial.println("DATA" + String(count) + ".csv already exist");
        Serial.end();
        count++;
        }
        else {
          gpsFile.open(pathName, O_CREAT | O_RDWR);
          gpsFile.println(labels_gps);
          gpsFile.close();
          Serial.begin(GPSBaud);
          Serial.println("a new file named " + strName + " has opened"); 
          Serial.end();
          gpsFileName = strName;
          isFile = true;
        }
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
   gpsFile.open(pathName, O_CREAT | O_RDWR); 
   Serial.begin(GPSBaud);
   Serial.println("coping data into " + gpsFileName + fileType);
   Serial.end();
   // if the car is stopping for more than 5 seconds, then there is no need to write the gps data (because they haven't changed)
   if (timeCount < writingTime) {
    gpsFile.print(gpsLatitude + ",");
    gpsFile.print(gpsLongitude + ",");
    gpsFile.print(gpsAltitude + ",");
    gpsFile.print(gpsSpeed + ",");
    gpsFile.print(CMG + ",");
    gpsFile.print(currentDate + ",");
    gpsFile.print(currentTime + ",");
    gpsFile.print(String(shuntVoltage) + ",");
    gpsFile.print(String(current) + ",");
    gpsFile.print(String(battaryVoltage1) + ",");
    gpsFile.println(String(power) + ",");
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
