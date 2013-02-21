#include "ZumoLights.h"

// constructor; takes arguments specifying whether to enable internal pull-up
// and the default state of the pin that the button is connected to
ZumoLights::ZumoLights(unsigned char pin1, unsigned char pin2, unsigned char pin3)
{
  _pin1 = pin1;
  _pin2 = pin2;
  _pin3 = pin3;

  _LED1 = false;
  _LED2 = false;
  _LED3 = false;
}

ZumoLights::ZumoLights()
{
  _pin1 = LED_1;
  _pin2 = LED_2;
  _pin3 = LED_3;

  _LED1 = false;
  _LED2 = false;
  _LED3 = false;
}


bool ZumoLights::DisplayChar(unsigned char output)
{
  if (output >= 8)
    return false;
  
  init();
  _LED1 = (output>>0) & 1;
  _LED2 = (output>>1) & 1;
  _LED3 = (output>>2) & 1;
  ShowLights(); 
}

void ZumoLights::ToggleLED1()
{
  init();
  _LED1 = !(_LED1);
  ShowLights();
}

void ZumoLights::ToggleLED2()
{
  init();
  _LED2 = !(_LED2);
  ShowLights();
}

void ZumoLights::ToggleLED3()
{
  init();
  _LED3 = !(_LED3); 
  ShowLights();
}

void ZumoLights::TurnOnLED1()
{
  init();
  _LED1 = true;
  ShowLights();
}

void ZumoLights::TurnOnLED2()
{
  init();
  _LED2 = true;
  ShowLights();
}

void ZumoLights::TurnOnLED3()
{
  init();
  _LED3 = true;
  ShowLights();
}
void ZumoLights::TurnOffLED1()
{
  init();
  _LED1 = false;
  ShowLights();
}

void ZumoLights::TurnOffLED2()
{
  init();
  _LED2 = false;
  ShowLights();
}

void ZumoLights::TurnOffLED3()
{
  init();
  _LED3 = false;
  ShowLights();
}

void ZumoLights::AllOff()
{
  init();
  _LED1 = false;
  _LED2 = false;
  _LED3 = false;
  ShowLights();
}

void ZumoLights::AllOn()
{
  init();
  _LED1 = true;
  _LED2 = true;
  _LED3 = true;
  ShowLights();
}

void ZumoLights::ShowLights()
{
  // Output all the current values to the LEDs
  
  if (_LED1)
    digitalWrite(_pin1, HIGH);
  else
    digitalWrite(_pin1, LOW);
    
  if (_LED2)
    digitalWrite(_pin2, HIGH);
  else
    digitalWrite(_pin2, LOW);
    
  if (_LED3)
    digitalWrite(_pin3, HIGH);
  else
    digitalWrite(_pin3, LOW);
}

// initializes I/O pin for use as LED outputs
void ZumoLights::init2()
{
  pinMode(_pin1, OUTPUT);
  pinMode(_pin2, OUTPUT);
  pinMode(_pin3, OUTPUT);
  
  _LED1 = false;
  _LED2 = false;
  _LED3 = false;
  digitalWrite(_pin1, LOW);
  digitalWrite(_pin2, LOW);
  digitalWrite(_pin3, LOW);
}