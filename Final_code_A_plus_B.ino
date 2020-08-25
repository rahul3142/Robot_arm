#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Simple_AHRS.h>
#include <Servo.h>

Servo base;
Servo arm1;
Servo arm2;
Servo jaw;


// Create LSM9DS1 board instance.
Adafruit_LSM9DS1     lsm(1000);  // Use I2C, ID #1000

// Create simple AHRS algorithm using the LSM9DS1 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS1 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS1(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}



void setup() {

base.attach(3);
arm1.attach(5);
arm2.attach(6);
jaw.attach(9);


  Serial.begin(115200);
  Serial.println(F("Adafruit LSM9DS1 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialise the LSM9DS1 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS1 ... check your connections
    Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  configureLSM9DS1();
 
}

void loop() {

  
  sensors_vec_t   orientation;

  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));
  }
  
  delay(100);
  
int basei = orientation.roll;
  int basem = map(basei, -90, 90, 0, 180); // o - upright, 100 - down, horizontal
  base.write(basem);

  int arm1i = orientation.pitch;
  int arm1m = map(arm1i, -90, 90, 0, 100); //120 - up, 0 -down
  arm1.write(arm1m);

  int pot1 = analogRead(A0);
  int pot1m = map(pot1, 0, 1023, 0, 120);
  arm2.write(pot1m);

  int pot2i = analogRead(A1);
  int pot2m = map(pot2i, 0, 1023, 70, 140);
  jaw.write(pot2m);
}
