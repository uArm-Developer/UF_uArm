/************************************************************************
* File Name          : uArmCalibration
* Author             : Evan
* Updated            : Evan
* Version            : V0.0.1
* Date               : 21 May, 2014
* Description        : uArm Calibration
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*************************************************************************/
#include <Servo.h>
#include <EEPROM.h>
#include <UF_uArm.h>

UF_uArm uarm;           // initialize the uArm library 

void setup() 
{
  uarm.init();          // initialize the uArm position
  detachServo();
}

void loop()
{
  uarm.calibration();   // if corrected, you can remove it
} 

void detachServo()
{
  uarm.detachServo(SERVO_L);
  uarm.detachServo(SERVO_R);
  uarm.detachServo(SERVO_ROT);
  uarm.detachServo(SERVO_HAND_ROT);
  uarm.detachServo(SERVO_HAND);
}
