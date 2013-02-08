#include "RAMBLERBot.h"

#define LED_PIN 13

// TODO:
// - Dependent on time step
// - ??

// constructor
RAMBLERBot::RAMBLERBot(): button_(ZUMO_BUTTON)
{
  pinMode(LED_PIN, OUTPUT);
  state_ = ESTOP;
  heading_ = 0;
  setAccel(10);

  headingAvg_.SetWindow(10);
}

void RAMBLERBot::init()
{
  Wire.begin();
  compass_.init();
  compass_.enableDefault();
  
  // Calibration values. Use the Calibrate example program to get the values for
  // your compass.
  // M min X: -644 Y: 49 Z: 1655 M max X: -27 Y: 820 Z: 1772

  // RAMBLER Uno
  //compass_.m_min.x = -444; compass_.m_min.y = 282; compass_.m_min.z = 1286;
  //compass_.m_max.x = -135; compass_.m_max.y = 670; compass_.m_max.z = 1568;
  
  // RAMBLER Leonardo
  compass_.m_min.x = 662; compass_.m_min.y = 241; compass_.m_min.z = 1509;
  compass_.m_max.x = 1024; compass_.m_max.y = 753; compass_.m_max.z = 1726;;
}

void RAMBLERBot::setAccel(int maxAccel)
{
  max_accel_per_loop_ = maxAccel;
}

// FAST LOOP: 50 Hz-ish
void RAMBLERBot::loop50Hz()
{
  compass_.read();
  heading_ = compass_.heading((LSM303::vector){0,-1,0})-180;
  headingAvg_.Push(heading_);
  
  // Check if the speed should accelerate or decelerate
  if (speedLimit(&motor_left_, motor_left_goal_))
  {
    motors_.setLeftSpeed(motor_left_);
  }
  
  // Check if the speed should accelerate or decelerate
  if (speedLimit(&motor_right_, motor_right_goal_))
  {
    motors_.setRightSpeed(motor_right_);
  }

}

// SLOW LOOP: 10 Hz-ish
void RAMBLERBot::loop10Hz()
{
  heading_curr_ = headingAvg_.AngleMean();
  
  // Check the Emergency Stop
  if (state_ != ESTOP && button_.isPressed())
  {
    index_ = 0;
    start_ = false;
    state_ = ESTOP;
    motors_.setRightSpeed(0);
    motors_.setLeftSpeed(0);
    motor_left_ = 0;
    motor_right_ = 0;
    motor_left_goal_ = 0;
    motor_right_goal_ = 0;
    buzzer_.playNote(NOTE_G(5),100,15);
    button_.waitForRelease();  // wait here for the button to be released
  }
  
  // Progress through the overall Robot States
  switch (state_)
  {
    case ESTOP:
      if (button_.isPressed())
      {
        state_ = ESTOP_TO_ACTIVE;
        button_.waitForRelease();  // wait here for the button to be released
      }
      break;
      
    case ESTOP_TO_ACTIVE:
      if (index_ < 22)
      {
        if (++index_ % 2 == 0)
        {
          buzzer_.playNote(NOTE_G(5),100,15);
        }
      }
      else
      {
        index_ = 0;
        state_ = PAUSE;
      }          
      break;
    
    case PAUSE:
      if (index_ < 10)
      {
        index_++;
      }
      else
      {
        index_ = 0;
        state_ = FORWARD;
      }          
      break;
      
    case FORWARD:
      if (index_ < 10)
      {
        index_++;
        motor_left_goal_ = 300;
        motor_right_goal_ = 300;
      }
      else if (index_ < 20)
      {
        index_++;
        motor_left_goal_ = 0;
        motor_right_goal_ = 0;
      }
      else
      {
        index_ = 0;
        state_ = TURN_LEFT;
      }
      break;
      
      
    case TURN_LEFT:
      if (!start_)
      {
        if (index_ < 1)
        {
          index_++;
        }
        else
        {
          index_ = 0;
          heading_start_ = heading_curr_;
          start_ = true;
          motor_right_goal_ = +100;
          motor_left_goal_ = -100;
        }          
      }
      else
      {
        if (AngleUtilities::angleDifference(heading_start_,heading_curr_) > 90)
        {
          motor_right_goal_ = 0;
          motor_left_goal_ = 0;
          motor_right_ = 0;
          motor_left_ = 0;
          motors_.setRightSpeed(0);
          motors_.setLeftSpeed(0);
          start_ = false;
          state_ = FORWARD;
        }
        else
        {       
          //Serial.print("Start: ");
          //Serial.print(heading_start_);
          //Serial.print(" Current: ");
          //Serial.print(heading_curr_);
          //Serial.print(" Offset: ");
          //Serial.println(AngleUtilities::angleDifference(heading_start_,heading_curr_));
        }
      }
      
      
      // while (true)
      // {
       // compass.read();
       // heading = compass.heading((LSM303::vector){0,-1,0})-180;        
       // Serial.print("Start: ");
       // Serial.print(startHeading);
       // Serial.print(" Current: ");
       // Serial.print(heading);
       // Serial.print(" Offset: ");
       // Serial.println(AngleUtilities::angleDifference(startHeading,heading));
      // }
      break;
      
      
    default:
      state_ = ESTOP;
      break;
  }
}


// Function to limit the speed according to the acceleration rate
//   speed: Current motor speed
//   speed_goal: Desired motor speed
// returns 
//   bool: True if the speed is changed
bool RAMBLERBot::speedLimit(int *speed, int speed_goal)
{
  int s = *speed;
  if (s < speed_goal)
  {
    if ((s + max_accel_per_loop_) < speed_goal )
      *speed = s + max_accel_per_loop_;
    else
      *speed = speed_goal;
    return true;
  }
  else if (s > speed_goal)
  {
    if ((s - max_accel_per_loop_) > speed_goal )
      *speed = s - max_accel_per_loop_;
    else
      *speed = speed_goal; 
    return true;
  }
  return false;
}
