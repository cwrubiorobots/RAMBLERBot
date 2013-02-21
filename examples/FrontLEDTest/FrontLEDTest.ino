/*
Testing the LEDs

EJ Kreinar
*/

// For whatever reason the Arduino IDE needs me to
// include ALL of these stupid libraries...
#include <LSM303.h>
#include <Wire.h>
#include <Pushbutton.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <AngleUtilities.h>
#include <LList.h>
#include <MovingAverage.h>
#include <RamblerAlgorithm.h>
#include <RAMBLERBot.h>

// This is the only library under test
#include <ZumoLights.h>

ZumoLights lights;

void setup()
{  
  Serial.begin(9600);
}

void loop()
{
  lights.AllOn();
  delay(500);
  lights.AllOff();
  delay(500);
  lights.ToggleLED1();
  delay(500);
  lights.ToggleLED1();
  delay(500);
  lights.ToggleLED2();
  delay(500);
  lights.ToggleLED2();
  delay(500);
  lights.ToggleLED3();
  delay(500);
  lights.DisplayChar(1);
  delay(500);
  lights.DisplayChar(2);
  delay(500);
  lights.DisplayChar(3);
  delay(500);
  lights.DisplayChar(4);
  delay(500);
  lights.DisplayChar(5);
  delay(500);
  lights.DisplayChar(6);
  delay(500);
  lights.DisplayChar(7);
  delay(500);
  lights.AllOff();
  delay(500);
}
