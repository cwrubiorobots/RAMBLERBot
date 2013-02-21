#ifndef ZumoLights_h
#define ZumoLights_h

#include <Arduino.h>

#define LED_1 11
#define LED_2 5
#define LED_3 4

class ZumoLights
{
  public:
    
    // constructor; takes arguments specifying whether to enable internal pull-up
    // and the default state of the pin that the button is connected to
    ZumoLights();
    ZumoLights(unsigned char pin1, unsigned char pin2, unsigned char pin3);

    // Encode the output integer on the 3 LEDs (0-7 binary encoding)
    bool DisplayChar(unsigned char output);
    
    // Various functions for zumo lights
    void ToggleLED1();
    void ToggleLED2();
    void ToggleLED3();
    
    void TurnOnLED1();
    void TurnOnLED2();
    void TurnOnLED3();
    
    void TurnOffLED1();
    void TurnOffLED2();
    void TurnOffLED3();
    
    void AllOff();
    void AllOn();
    
    
  private:
    unsigned char _pin1;
    unsigned char _pin2;
    unsigned char _pin3;
    bool _LED1;
    bool _LED2;
    bool _LED3;
    
    void ShowLights();
    
    inline void init()
    {
      static boolean initialized = false;

      if (!initialized)
      {
        initialized = true;
        init2();
      }
    }
    
    // initializes I/O pins for use as outputs
    void init2();
};

#endif