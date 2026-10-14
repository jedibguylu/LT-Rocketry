#include <cmath>
#include <iostream>
#include <string>
#include <numbers>

//TARGET COORDINATES
std::vector<double> targetPoint = {0, 0};

//GPS Setup - https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/
#include <TinyGPS++.h> //when importing its just called TinyGPSPlus by Mikal Hart
#include <SoftwareSerial.h>

static const int GRX = 4, GTX = 3; //TX pin on module is 4, RX pin on module is 3

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GRX, GTX); //Serial connection to device

//Compass Setup - https://projecthub.arduino.cc/mircemk/arduino-analog-digital-compass-with-hmc5883l-sensor-3ab86f
#include <HMC5883L.h> //when importing its called Grove_3Axis_Digital_Compass_HMC5883L
#include <Wire.h>

HMC5883L compass;

//Motor Controller Setup
static const int in1Pin = D8, in2Pin = D7, in3Pin = D6, in4Pin = D5;

//SD Writer Setup
#include <SPI.h>
#include <SD.h>

const char* fileName = "log.csv"


//---------------ADDED VARIABLES FOR TESTING POURPOSES------------------

//----------------------------------------------------------------------

int alterControls(int angle, double distance){
  if (std::abs(distance) > 0.00001){ //should be ~1m
      if (angle < 0 && std::abs(angle) > 5) {
          DitgalWrite(D6, HIGH);
          DitgalWrite(D5, LOW);
          DitgalWrite(D8, LOW);
          DitgalWrite(D7,LOW);
          return -1;
      } else if (angle > 0 && std::abs(angle) > 5) {
          DitgalWrite(D8, HIGH);
          DitgalWrite(D7,LOW);
          DitgalWrite(D6, LOW);
          DitgalWrite(D5, LOW);
          return 1;
      } else {
          DitgalWrite(D8, HIGH);
          DitgalWrite(D7, LOW);
          DitgalWrite(D6, HIGH);
          DitgalWrite(D5, LOW);
          return 0;
      }
  } else {
      DitgalWrite(D8, LOW);
      DitgalWrite(D7, LOW);
      DitgalWrite(D6, LOW);
      DitgalWrite(D5, LOW);
      return 0;
  }
}

int calcAngle(double boatX, double boatY, double targetX, double targetY, int heading) {
    double radConvert = 3.14159 / 180.0; //from deg to rad
    double degConvert = 180.0 / 3.14159; //from rad to deg
    
    // https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
    double dx = std::cos(targetX * radConvert) * std::sin((targetY - boatY) * radConvert);
    double dy = std::cos(boatX * radConvert) * std::sin(targetX * radConvert) - std::sin(boatX * radConvert) * std::cos(targetX * radConvert) * std::cos((targetY - boatY) * radConvert);
    
    std::cout << dx << "\n";
    std::cout << dy << "\n";

    // Calculate angle using atan2(y, x)
    double bearing = std::atan2(dx, dy) * degConvert;
    
    std::cout << bearing << "\n";

    // Normalize bearing to [0, 360)
    int course = static_cast<int>(std::fmod(bearing + 360.0, 360.0));

    std::cout << course << "\n";

    // Calculate turn difference relative to current heading (-180 to +180)
    int difference = course - heading;
    while (difference > 180)  difference -= 360;
    while (difference < -180) difference += 360;

    return difference;
}

double calcDist(double boatX, double boatY, double targetX, double targetY){
    double xDist = targetX - boatX;
    double yDist = targetY - boatY;
    double distance = std::sqrt((xDist * xDist) + (yDist * yDist));
    return distance;
}

std::vector<double> getCoords() {
  double lat, lon;
  
  if  (gps.location.isUpdated()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }

  std::vector<double> point = {lat, lon};
  
  return point;
}

int getHeading() {
 return 0;
}

void storeData(int time, std::vector<double> coords, double distance, int heading, int angle, int turn status, String name){
  String dataString = "";
  dataString += time;
  dataString += " [Coords: ";
  dataString += coords[0];
  dataString += " deg, ";
  dataString += coords[1];
  dataString += " deg | Dist: ";
  dataString += distance;
  dataString += " (unk)| Heading: ";
  dataString += heading;
  dataString += "deg | Angle: ";
  dataString += angle;
  dataString += " deg diff | Turn satus: ";
  dataString += status;

  File dataFile = SD.open(name, FILE_WRITE);

	if (dataFile) {
		dataFile.println(dataString);
		dataFile.close(); // Closing flushes data to physical storage

		Serial.print("Log Successful");
	} else {
		Serial.println("Error opening file for writing!");
	}

	return 0;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  //GPS
  gpsSerial.begin(9600);

  //Compass
  Wire.begin(); //Initializes I2C Compass Pins
  compass.setMeasurementMode(MEASUREMENT_CONTINUOUS);

  //Motor Controller
  PinMode(in1Pin, OUTPUT);  
  PinMode(in2Pin, OUTPUT);   
  PinMode(in3Pin, OUTPUT);   
  PinMode(in4Pin, OUTPUT);

  //SD Writer
  if (!SD.begin(chipSelect)) {
		Serial.println("Initialization failed! Check connections or SD card.");
		while (true); // Halt execution
	}
	Serial.println("SD card initialized successfully.");

	if (!SD.exists(fileName)) { // Write CSV header row if the file doesn't exist yet
		File dataFile = SD.open(fileName, FILE_WRITE);
		if (dataFile) {
			dataFile.println("Log Created");
			dataFile.close();
			Serial.println("New log created.");
		}
	}
}

void loop() {
  std::vector<double> currentPoint = getCoords();
  int currentHeading = getHeading();
  int angle = calcAngle(currentPoint[0], currentPoint[1], targetPoint[0], targetPoint[1], currentHeading);
  double distance = calcDist(currentPoint[0], currentPoint[1], targetPoint[0], targetPoint[1]);
  int controlStatus = alterControls(angle, distance);
  storeData(gps.time.value(), currentPoint, distance, currentHeading, angle, controlStatus, fileName);
  
  delay(100);
}
