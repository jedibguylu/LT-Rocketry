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
static const int in1Pin = D8;
static const int in2Pin = D7;
static const int in3Pin = D6;
static const int in4Pin = D5;

//SD Writer Setup
#include <SPI.h>
#include <SD.h>

const char* fileName = "log.csv"

//--------------------------------------------------------------

void alterControls(int angle, int distance){
  return 0;
}

int calcAngle(double boatX, double boatY, double targetX, double targetY, int heading) {
  return difference;
}

double calcDist(double boatX, double boatY, double targetX, double targetY){
  return distance;
}

std::vector<double> getCoords() {
 return point; //array
}

void getHeading() {
 ...
}

void storeData(int time, std::vector<double> coords, double distance, int heading, int angle, int turn status, File file, String name){
 return 0;
?

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
  //all the functions go in here
}
