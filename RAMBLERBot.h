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
#include <ZumoLights.h>
#include <ZumoWhiskers.h>
#include <RamblerSerialRcv.h>
#include <RamblerAlgorithm.h>

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
    
    // 10 Hz Loop Offset from the Main Loop
    // Could do sensor reading in here if necessary?? I dont know
    void loop10HzOffset();
    
    void setAccel(int maxAccel);
    
    // Variable returning functions
    int getMotorLeft() {return motor_left_;}
    int getMotorRight() {return motor_right_;}
    int getHeading() {return heading_curr_;}
    RamblerAlgorithm::RamblerState getCockroachState() {return brain.getState();} //RamblerAlgorithm::STRAIGHT; }//
    
  private:
    //Member Functions
    bool speedLimit(int *speed, int speed_goal);
  
    RamblerAlgorithm brain;
  
    // Member classes for hardware abstraction
    ZumoBuzzer buzzer_;
    ZumoMotors motors_; 
    ZumoLights lights_;
    Pushbutton button_;
    ZumoWhiskers whiskers_;
    RamblerSerialRcv receiver_;
    // LSM303 compass_;
    
    // Helper classes and variables
    int heading_;
    int heading_curr_;
    int heading_start_;
    MovingAverage headingAvg_;
    int index_;
    int count50Hz_;
    bool start_;
    int rand1_;
    int rand2_;
    int rand3_;
    unsigned long time_;
    
    // Member variables
    float v_;
    float w_;
    int max_accel_per_loop_;
    int motor_left_;
    int motor_right_;
    int motor_left_goal_;
    int motor_right_goal_;
    int ant_right_;
    int ant_left_;
  
    enum rambler_state_t {
      ESTOP = 0,
      ESTOP_TO_ACTIVE,
      COCKROACH,
      TEST_FORWARD,
      TEST_WALLFOLLOW,
      TEST_PIVOT,
      TEST_END,
      PAUSE
    };
    rambler_state_t state_;
  
};

#endif
