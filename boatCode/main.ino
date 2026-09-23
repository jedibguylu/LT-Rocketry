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
static const int Left1 = 8; //left and right dont mean anything until we connect them
static const int Left2 = 7;
static const int Right1 = 6;
static const int Right2 = 5;

void setup() {
  Serial.begin(9600);

  //GPS
  gpsSerial.begin(9600);

  //Compass
  Wire.begin(); //Initializes I2C Compass Pins
  compass.setMeasurementMode(MEASUREMENT_CONTINUOUS);

  //Motor Controller
  pinMode(Left1, OUTPUT);
  pinMode(Left2, OUTPUT);
  pinMode(Right1, OUTPUT);
  pinMode(Right2, OUTPUT);
}

void loop() {
  //all the functions go in here
}
