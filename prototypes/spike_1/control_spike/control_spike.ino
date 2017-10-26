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
#include "controller.h"

#define P_TUNING 1
#define I_TUNING 0
#define D_TUNING 0.5

AbstractServo servo(1100, 1940, 2, -100.0f, 100.0f);

Controller * control = new Controller();

void setup() {
  // put your setup code here, to run once:

  SensorHub::init();
  Serial.begin(9600);
  Serial1.begin(57600);

  servo.setIs3DMotor(true);

  servo.enable();

  control->init(P_TUNING, I_TUNING, D_TUNING);

  control->setSetpoint(0);

  control->setCurrentValue(0);

  control->applySetpointLimits(20.0f, -20.0f);

  servo.setPercentLimits(-20, 20);

}

void loop() {
  // put your main code here, to run repeatedly:

  SensorHub::update();

  servo.update();
  
//  servo.setPower(sin( millis()/(float)500 ) * 20 );

  

  control->setP(P_TUNING);

  control->setI(I_TUNING);

  control->setD(D_TUNING);

  control->setSetpoint(0);

  control->setCurrentValue(SensorHub::getGyro().z);

  control->update();

  servo.setAcceleration(-control->getOutput());

  //servo.setPower(-control->getOutput());

  Serial1.print(control->getOutput());

  Serial1.print(" ");

/*  if(millis() / 2000 <= 1000)
  {
    servo.setPower(20);
  }
  else
  {

    servo.setPower(0);
    
  }*/
  
}


