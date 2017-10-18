#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "kalmanfilter.h"
#include <math.h>
#include <Servo.h>
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <MPU6050.h>
#include "dimensionals.h"
#include "quatops.h"c
#include "sensorhub.h"
#include "AbstractServo.h"

AbstractServo servo(1100, 1940, 2, -100.0f, 100.0f);

void setup() {
  // put your setup code here, to run once:

  SensorHub::init();
  Serial.begin(9600);

  servo.setIs3DMotor(true);

  servo.enable();

}

void loop() {
  // put your main code here, to run repeatedly:

  SensorHub::update();

  servo.update();
  
//  servo.setPower(sin( millis()/(float)500 ) * 20 );

  if(millis() / 2000 <= 1000)
  {
    servo.setPower(20);
  }
  else
  {

    servo.setPower(0);
    
  }
  
}


