#include "ZumoWhiskers.h"

// constructor; takes arguments specifying whether to enable internal pull-up
// and the default state of the pin that the button is connected to
ZumoWhiskers::ZumoWhiskers()
{
  pinLeft_ = A0;
  pinLeft_ = A1;
}

void ZumoWhiskers::ReadWhiskers()
{
  whiskerLeft_ = analogRead(pinLeft_);
  whiskerRight_ = analogRead(pinRight_);
}

void ZumoWhiskers::GetWhiskerValues(int *right, int *left)
{
  //TODO: Interpret the raw analog input
  
  *right = whiskerRight_;
  *left = whiskerLeft_;
}