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

void setup() {
  Serial.begin(9600);

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
}

void loop() {
  //all the functions go in here
}
