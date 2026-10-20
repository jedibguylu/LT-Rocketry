#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <HMC5883L.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

struct Point {
  double lat;
  double lng;
};

//TARGET COORDINATES
Point targetPoint = {41.9603426312413, -87.63466132170615}; //Chicago
//-------------TESTING VARIABLES----------------
const bool testMode = true;
Point testBoat = {41.82830311848214, -86.70505674169225}; //New Buffalo
int testHeading = 360;
//----------------------------------------------
//Math Variables
const double radConvert = 3.14159 / 180.0; //from deg to rad
const double degConvert = 180.0 / 3.14159; //from rad to deg
const double R = 6371000.0; // Earths radius in meters

//GPS Setup - https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/
static const int GRX = 4, GTX = 3; //TX pin on module is 4, RX pin on module is 3

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GRX, GTX); //Serial connection to device

//Compass Setup - https://projecthub.arduino.cc/mircemk/arduino-analog-digital-compass-with-hmc5883l-sensor-3ab86f
#include <HMC5883L.h> //when importing its called Grove_3Axis_Digital_Compass_HMC5883L
//#include <Wire.h>

HMC5883L compass;

//Motor Controller Setup
static const int in1Pin = 8, in2Pin = 7, in3Pin = 6, in4Pin = 5;

//SD Writer Setup
//#include <SPI.h>
#include <SD.h>
const int chipSelect = 10;
const char* fileName = "log.csv";

int alterControls(int angle, double distance){
  if (abs(distance) > 1){ //should be ~1m
      if (angle < 0 && abs(angle) > 5) {
          digitalWrite(6, HIGH);
          digitalWrite(5, LOW);
          digitalWrite(8, LOW);
          digitalWrite(7,LOW);
          return -1;
      } else if (angle > 0 && abs(angle) > 5) {
          digitalWrite(8, HIGH);
          digitalWrite(7,LOW);
          digitalWrite(6, LOW);
          digitalWrite(5, LOW);
          return 1;
      } else {
          digitalWrite(8, HIGH);
          digitalWrite(7, LOW);
          digitalWrite(6, HIGH);
          digitalWrite(5, LOW);
          return 0;
      }
  } else {
      digitalWrite(8, LOW);
      digitalWrite(7, LOW);
      digitalWrite(6, LOW);
      digitalWrite(5, LOW);
      return 0;
  }
}

int calcAngle(double boatLat, double boatLon, double targetLat, double targetLon, int heading) {
    // https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
    double dx = cos(targetLat * radConvert) * sin((targetLon - boatLon) * radConvert);
    double dy = cos(boatLat * radConvert) * sin(targetLat * radConvert) - sin(boatLat * radConvert) * cos(targetLat * radConvert) * cos((targetLon - boatLon) * radConvert);
    
    //cout << dx << "\n";
    //cout << dy << "\n";

    // Calculate angle using atan2(y, x)
    double bearing = (atan2(dx, dy) * degConvert) + 6;
    
    //cout << bearing << "\n";

    // Normalize bearing to [0, 360)
    int course = static_cast<int>(fmod(bearing + 360.0, 360.0));

    //cout << course << "\n";

    // Calculate turn difference relative to current heading (-180 to +180)
    int difference = course - heading;
    while (difference > 180)  difference -= 360;
    while (difference < -180) difference += 360;

    return difference;
}

double calcDist(double boatLat, double boatLon, double targetLat, double targetLon){
    double lat1 = boatLat * radConvert;
    double lat2 = targetLat * radConvert;
    double dLat = (targetLat - boatLat) * radConvert;
    double dLon = (targetLon - boatLon) * radConvert;

    // Haversine calculation
    double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1) * cos(lat2) *
               sin(dLon / 2.0) * sin(dLon / 2.0);
               
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c; // Distance in meters
}

Point getCoords() {
  Point p;
  
  if (!testMode){
    if (gps.location.isUpdated()) {
    p.lat = gps.location.lat();
    p.lng = gps.location.lng();
    }
  } else {
    p.lat = testBoat.lat;
    p.lng = testBoat.lng;
  }
  
  return p;
}

int getHeading() {
 return testHeading;
}

void storeData(int time, Point coords, double distance, int heading, int angle, int turnStatus, String name){
  String dataString = "";
  dataString += time;
  dataString += " [Coords: ";
  dataString += coords.lat;
  dataString += " deg, ";
  dataString += coords.lng;
  dataString += " deg | Dist: ";
  dataString += distance;
  dataString += " meters| Heading: ";
  dataString += heading;
  dataString += "deg | Angle: ";
  dataString += angle;
  dataString += " deg diff | Turn satus: ";
  dataString += turnStatus;

  if (!testMode){
    File dataFile = SD.open(name, FILE_WRITE);

    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close(); // Closing flushes data to physical storage
      Serial.println(dataString);
    } else {
      Serial.println("Error opening file for writing!");
    }
  } else {
    Serial.println(dataString);
  }
  
	return 0;
}

void altitudeLimiter(){
  delay(600000); //10 min
  int groundlvl = gps.altitude.meters();
  
  while (gps.altitude.meters() < gps.altitude.meters() + 30){
    delay(500);
  }

  while (gps.altitude.meters() > gps.altitude.meters() + 5){
    delay(500);
  }

  delay(60000); //1 min
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  Serial.println("Booting...");
  //GPS
  gpsSerial.begin(9600);
  Serial.println("GPS Initialized");
  //Compass
  Wire.begin(); //Initializes I2C Compass Pins
  compass.setMeasurementMode(MEASUREMENT_CONTINUOUS);
  Serial.println("Compass Initialized");

  //Motor Controller
  pinMode(in1Pin, OUTPUT);  
  pinMode(in2Pin, OUTPUT);   
  pinMode(in3Pin, OUTPUT);   
  pinMode(in4Pin, OUTPUT);
  Serial.println("Motors Initialized");

  //SD Writer
  if (!testMode){
    if (!SD.begin(chipSelect)) {
      Serial.println(F("SD Card initialization failed!"));
      while(!SD.begin(chipSelect)){}
    } else {
      Serial.println(F("SD Card initialized."));
      if (!SD.exists(fileName)) {
        File dataFile = SD.open(fileName, FILE_WRITE);
        if (dataFile) {
          dataFile.println("Log Created");
          dataFile.close();
          Serial.println(F("New log created."));
        }
      }
    }
  }
  Serial.println("Boot complete.");

  Serial.println("Holding for altitude...");
  if(!testMode){
	  altitudeLimiter();
  }
  Serial.println("Loop conditions met. Starting main loop.");
}

void loop() {
  int time = 0;
  if (!testMode) {
    time = gps.time.value();
  } else {
    time = 6767;
  }
  Point boatPoint = getCoords();
  int currentHeading = getHeading();
  int angle = calcAngle(boatPoint.lat, boatPoint.lng, targetPoint.lat, targetPoint.lng, currentHeading);
  double distance = calcDist(boatPoint.lat, boatPoint.lng, targetPoint.lat, targetPoint.lng);
  int controlStatus = alterControls(angle, distance);
  storeData(time, boatPoint, distance, currentHeading, angle, controlStatus, fileName);
  
  delay(500);
}
