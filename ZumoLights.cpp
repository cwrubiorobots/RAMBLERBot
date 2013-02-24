#include "ZumoLights.h"

// constructor; takes arguments specifying whether to enable internal pull-up
// and the default state of the pin that the button is connected to
ZumoLights::ZumoLights(unsigned char pin1, unsigned char pin2, unsigned char pin3)
{
  pin1_ = pin1;
  pin2_ = pin2;
  pin3_ = pin3;

  LED1_ = false;
  LED2_ = false;
  LED3_ = false;
}

ZumoLights::ZumoLights()
{
  pin1_ = LED_1;
  pin2_ = LED_2;
  pin3_ = LED_3;

  LED1_ = false;
  LED2_ = false;
  LED3_ = false;
}


bool ZumoLights::DisplayChar(unsigned char output)
{
  if (output >= 8)
    return false;
  
  init();
  LED1_ = (output>>0) & 1;
  LED2_ = (output>>1) & 1;
  LED3_ = (output>>2) & 1;
  ShowLights(); 
  return true;
}

void ZumoLights::ToggleLED1()
{
  init();
  LED1_ = !(LED1_);
  ShowLights();
}

void ZumoLights::ToggleLED2()
{
  init();
  LED2_ = !(LED2_);
  ShowLights();
}

void ZumoLights::ToggleLED3()
{
  init();
  LED3_ = !(LED3_); 
  ShowLights();
}

void ZumoLights::TurnOnLED1()
{
  init();
  LED1_ = true;
  ShowLights();
}

void ZumoLights::TurnOnLED2()
{
  init();
  LED2_ = true;
  ShowLights();
}

void ZumoLights::TurnOnLED3()
{
  init();
  LED3_ = true;
  ShowLights();
}
void ZumoLights::TurnOffLED1()
{
  init();
  LED1_ = false;
  ShowLights();
}

void ZumoLights::TurnOffLED2()
{
  init();
  LED2_ = false;
  ShowLights();
}

void ZumoLights::TurnOffLED3()
{
  init();
  LED3_ = false;
  ShowLights();
}

void ZumoLights::AllOff()
{
  init();
  LED1_ = false;
  LED2_ = false;
  LED3_ = false;
  ShowLights();
}

void ZumoLights::AllOn()
{
  init();
  LED1_ = true;
  LED2_ = true;
  LED3_ = true;
  ShowLights();
}

void ZumoLights::ShowLights()
{
  // Output all the current values to the LEDs
  
  if (LED1_)
    digitalWrite(pin1_, HIGH);
  else
    digitalWrite(pin1_, LOW);
    
  if (LED2_)
    digitalWrite(pin2_, HIGH);
  else
    digitalWrite(pin2_, LOW);
    
  if (LED3_)
    digitalWrite(pin3_, HIGH);
  else
    digitalWrite(pin3_, LOW);
}

// initializes I/O pin for use as LED outputs
void ZumoLights::init2()
{
  pinMode(pin1_, OUTPUT);
  pinMode(pin2_, OUTPUT);
  pinMode(pin3_, OUTPUT);
  
  LED1_ = false;
  LED2_ = false;
  LED3_ = false;
  digitalWrite(pin1_, LOW);
  digitalWrite(pin2_, LOW);
  digitalWrite(pin3_, LOW);
}