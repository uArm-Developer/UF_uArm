/************************************************************************
* File Name          : MotionDemo
* Author             : Evan
* Updated            : Evan
* Version            : V0.0.2
* Date               : 9 June, 2014
* Description        :
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*************************************************************************/
#include <EEPROM.h>
#include <UF_uArm.h>

UF_uArm uarm;           // initialize the uArm library 

void setup() 
{
  uarm.init();          // initialize the uArm position
  uarm.setServoSpeed(SERVO_R, 50);  // 0=full speed, 1-255 slower to faster
  uarm.setServoSpeed(SERVO_L, 50);  // 0=full speed, 1-255 slower to faster
  uarm.setServoSpeed(SERVO_ROT, 50); // 0=full speed, 1-255 slower to faster
  delay(500);
}

void loop()
{
  motion();
  motionReturn();
} 

void motion()
{
  uarm.setPosition(60, 0, 0, 0);    // stretch out
  delay(400);
  uarm.setPosition(60, -45, 0, 0);  // down
  uarm.gripperCatch();               // catch
  delay(400);
  uarm.setPosition(60, 0, 0, 0);    // up
  delay(400);
  uarm.setPosition(60, 0, 25, 0);   // rotate
  delay(400);
  uarm.setPosition(60, -45, 25, 0); // down
  delay(400);
  uarm.gripperRelease();             // release
  delay(100);
  uarm.setPosition(60, 0, 25, 0);   // up
  delay(400);
  uarm.setPosition(0, 0, 25, 0);     
  delay(400);
  uarm.gripperDirectDetach();        // direct detach 
  delay(500);
}

void motionReturn()
{
  uarm.setPosition(60, 0, 25, 0);    // stretch out
  delay(400);
  uarm.setPosition(60, -45, 25, 0);  // down
  uarm.gripperCatch();                // catch
  delay(400);
  uarm.setPosition(60, 0, 25, 0);    // up
  delay(400);
  uarm.setPosition(60, 0, 0, 0);     // rotate
  delay(400);
  uarm.setPosition(60, -45, 0, 0);   // down
  delay(400);
  uarm.gripperRelease();              // release
  delay(100);
  uarm.setPosition(60, 0, 0, 0);     // up
  delay(400);
  uarm.setPosition(0, 0, 0, 0);       // original position
  delay(400);
  uarm.gripperDirectDetach();         // direct detach 
  delay(500);
}
