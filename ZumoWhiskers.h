#ifndef ZumoWhiskers_h
#define ZumoWhiskers_h

#include <Arduino.h>
#include <MovingAverage.h>

#define WHISKER_RIGHT A2
#define WHISKER_LEFT A3

// WHISKER_LOWPASS_NUM:
// Constant that determines how many iterations to low-pass the 
//  analog input between calibrations
#define WHISKER_LOWPASS_NUM 10

// WHISKER_RESTING_POINT:
//  Constant that determines how far the calibration is 
//  set from the whisker resting point
#define WHISKER_RESTING_POINT 10

class ZumoWhiskers
{
  public:
    
    // constructor; takes arguments specifying whether to enable internal pull-up
    // and the default state of the pin that the button is connected to
    ZumoWhiskers();

    // Encode the output integer on the 3 LEDs (0-7 binary encoding)
    bool DisplayChar(unsigned char output);
    
    // Read and store the Whisker Values
    void ReadRawWhiskers(int *right, int *left);
    
    // Return the whisker values
    void GetWhiskerValues(int *right, int *left);
    
    void ResetCalibration();
    
  private:
    bool CalibrateWhisker(int offset, MovingAverage *avg, int *calib);
    
    int pinLeft_;
    int pinRight_;
    
    int whiskerLeft_;
    int whiskerRight_;
    
    int calibLeft_;
    int calibRight_;
    bool calibLeftDone_;
    bool calibRightDone_;
    
    MovingAverage avgRight_;
    MovingAverage avgLeft_;
};

#endif