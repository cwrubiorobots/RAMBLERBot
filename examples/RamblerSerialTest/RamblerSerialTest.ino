#include <AngleUtilities.h>
#include <LList.h>
#include <MersenneTwister.h>
#include <MovingAverage.h>
#include <RamblerAlgorithm.h>
#include <RAMBLERBot.h>
#include <RamblerSerialRcv.h>
#include <ZumoLights.h>
#include <ZumoWhiskers.h>

#include <ZumoMotors.h>

#include <ZumoBuzzer.h>

#include <Pushbutton.h>

#include <Wire.h>

#include <LSM303.h>

#include <EEPROM.h>

/*
Testing the LEDs

EJ Kreinar
*/

// For whatever reason the Arduino IDE needs me to
// include ALL of these stupid libraries...

RamblerSerialRcv receiver;

void setup()
{  
  Serial.begin(9600);
}

void loop()
{
  receiver.ParseSerial();
}
