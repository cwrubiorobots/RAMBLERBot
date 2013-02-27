#include "ZumoWhiskers.h"

// constructor; takes arguments specifying whether to enable internal pull-up
// and the default state of the pin that the button is connected to
ZumoWhiskers::ZumoWhiskers()
{
  // STORE THE PIN VALUES
  pinRight_ = WHISKER_RIGHT;
  pinLeft_ = WHISKER_LEFT;
  
  // store the initial "calibration" or set point for each antenna
  calibRight_ = 1000;
  calibLeft_ = 1000;
  calibRightDone_ = false;
  calibLeftDone_ = false;
  
  avgRight_.SetWindow(WHISKER_LOWPASS_NUM);
  avgLeft_.SetWindow(WHISKER_LOWPASS_NUM);
  
}

void ZumoWhiskers::ReadRawWhiskers(int *right, int *left)
{
  // Read the raw whisker
  whiskerLeft_ = analogRead(pinLeft_);
  whiskerRight_ = analogRead(pinRight_);
  
  // Output debugging stuff
  // Serial.print("Left: ");
  // Serial.print(whiskerLeft_);
  // Serial.print("  Right: ");
  // Serial.print(whiskerRight_);
  
  *right = whiskerRight_;
  *left = whiskerLeft_;
}

void ZumoWhiskers::GetWhiskerValues(int *right, int *left)
{
  // Reads the raw whisker from the analog pins and then 
  // interprets from the calibration procedure
  whiskerLeft_ = analogRead(pinLeft_);
  whiskerRight_ = analogRead(pinRight_);
  
  int roff = whiskerRight_ - calibRight_;
  int loff = whiskerLeft_ - calibLeft_;
  
  // Handle the situation when the whisker value
  // is less than the calibration point
  if (roff < 0)
  {
    // Calibrate if Necessary
    if (!calibRightDone_)
      calibRightDone_ = CalibrateWhisker(roff, &avgRight_, &calibRight_);
    roff = 0;
  }
  if (loff < 0)
  {
    // Calibrate if necessary
    if (!calibLeftDone_)
      calibLeftDone_ = CalibrateWhisker(loff, &avgLeft_, &calibLeft_);
    loff = 0;
  }
  
  *right = roff;
  *left = loff;
}

void ZumoWhiskers::ResetCalibration()
{
  calibRight_ = 1000;
  calibLeft_ = 1000;
  calibRightDone_ = false;
  calibLeftDone_ = false;
}

bool ZumoWhiskers::CalibrateWhisker(int offset, MovingAverage *avg, int *calib)
{
  // Sure, I COULD have just decided to set the calibration point
  // at the first analog input + WHISKER_RESTING_POINT ...
  // ... But this way, maybe we can change the calibration on the fly eventually??
  avg->Push(offset);
  if ((avg->NumDataValues() >= WHISKER_LOWPASS_NUM))
  {
    if (avg->Mean() < -WHISKER_RESTING_POINT)
    {
      *calib = *calib + 0.75 * avg->Mean();
      avg->Clear();
      return false;
    }
    else
    {
      *calib = *calib + WHISKER_RESTING_POINT + avg->Mean();
      return true;
    }
  }
  return false;  
}