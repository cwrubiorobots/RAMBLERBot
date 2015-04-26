/*
Rambler Cockroach Robot: EMAE 477

EJ Kreinar
Charles Hart
David Chrzanowski
*/

#include <RAMBLERBot.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
//#include "RamblerSerialRcv.h"
#include <Wire.h>
//#include <LSM303.h>
#include <AngleUtilities.h>
#include <EEPROM.h>
#include <Servo.h>


RAMBLERBot robot;
Servo mantisClaw;
int mantisClawPin = 11;

int count = 0;
unsigned long newTime = 0;
unsigned long oldTime = 0;
unsigned long oldLoopTime = 0;
unsigned long duration = 0;

void setup()
{  
  Serial.begin(9600);
  
  pinMode(mantisClawPin, OUTPUT);
  mantisClaw.attach(mantisClawPin);

  robot.init();
}

void loop()
{
  newTime = millis();
  if ((newTime-oldTime) >= 20)
  {
    // ****** 50 HZ LOOP ******
    //Serial.println("Loop");
    oldTime = newTime;
    robot.loop50Hz();
    
    if (++count >= 5)
    {
      // ****** 10 HZ LOOP ******
      count = 0;
      duration = newTime-oldLoopTime;
      oldLoopTime = newTime; 
      robot.loop10Hz();
    }
  }  
}
