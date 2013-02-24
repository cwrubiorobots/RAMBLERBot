#ifndef ZumoWhiskers_h
#define ZumoWhiskers_h

#include <Arduino.h>

class ZumoWhiskers
{
  public:
    
    // constructor; takes arguments specifying whether to enable internal pull-up
    // and the default state of the pin that the button is connected to
    ZumoWhiskers();

    // Encode the output integer on the 3 LEDs (0-7 binary encoding)
    bool DisplayChar(unsigned char output);
    
    // Read and store the Whisker Values
    void ReadWhiskers();
    
    // Return the whisker values
    void GetWhiskerValues(int *right, int *left);
    
  private:
    int pinLeft_;
    int pinRight_;
    
    int whiskerLeft_;
    int whiskerRight_;
};

#endif