#ifndef RAMBLERBot_h
#define RAMBLERBot_h

#include <Arduino.h>
#include <AngleUtilities.h>
#include <LList.h>
#include <MovingAverage.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>
#include <Wire.h>
#include <LSM303.h>

// RAMBLERBot:
// Implements the RAMBLER Algorithm for the Arduino platform
class RAMBLERBot
{
  public:
    
    // constructor
    // TODO: Maybe provide a parameter for loop rate??
    RAMBLERBot();
    
    void init();
    
    // 100 Hz Loop
    void loop50Hz();
    
    // 10 Hz Loop
    void loop10Hz();
    
    void setAccel(int maxAccel);
    
    // Variable returning functions
    int getMotorLeft() {return motor_left_;}
    int getMotorRight() {return motor_right_;}
    int getHeading() {return heading_curr_;}
    
  private:
    //Member Functions
    bool speedLimit(int *speed, int speed_goal);
  
    // Member classes for hardware abstraction
    ZumoBuzzer buzzer_;
    ZumoMotors motors_; 
    Pushbutton button_;
    LSM303 compass_;
    
    // Helper classes and variables
    int heading_;
    int heading_curr_;
    int heading_start_;
    MovingAverage headingAvg_;
    int index_;
    bool start_;
    
    // Member variables
    int max_accel_per_loop_;
    int motor_left_;
    int motor_right_;
    int motor_left_goal_;
    int motor_right_goal_;
  
    enum rambler_state_t {
      ESTOP = 0,
      ESTOP_TO_ACTIVE,
      FORWARD,
      TURN_LEFT,
      PAUSE
    };
    rambler_state_t state_;
  
};

#endif