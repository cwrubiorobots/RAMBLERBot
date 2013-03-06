#include "RAMBLERBot.h"
#include <EEPROM.h>
#include "MersenneTwister.h"

#define LED_PIN 13

#define USE_NEW_SEED 1
#define USE_MERSENNE 0
#define SEED_ADDR_HIGH 0
#define SEED_ADDR_LOW 1

// TODO:
// - Dependent on time step
// - ??

// constructor

int fabs(int val)
{
  if (val < 0)
    return -1*val;
  else
    return val;
}

void SeedRandomGenerator()
{
  // SEED RANDOM GENERATOR
  
  // Read EEPROM
  byte oldSeedHigh = EEPROM.read(SEED_ADDR_HIGH);// << 8 + EEPROM.read(SEED_ADDR_LOW);
  byte oldSeedLow = EEPROM.read(SEED_ADDR_LOW);
  int seed = (oldSeedHigh << 8) + oldSeedLow;
  
  // Increment new seed or use the old one??
  if (USE_NEW_SEED)
    seed +=1;
  
  // Seed
  //init_genrand(static_cast<unsigned long>(seed));
  randomSeed(seed);
  Serial.print("Random Seed: ");
  Serial.println(seed);
  
  // Write back into EEPROM
  oldSeedHigh = seed >> 8;
  oldSeedLow = seed;
  EEPROM.write(SEED_ADDR_LOW, oldSeedLow);
  EEPROM.write(SEED_ADDR_HIGH, oldSeedHigh);
}

int GetRandomValue()
{
  //double r = genrand_real1();
  return random(1001);//int(genrand_real1()*1000);
}

RAMBLERBot::RAMBLERBot(): button_(ZUMO_BUTTON)
{
  pinMode(LED_PIN, OUTPUT);
  state_ = ESTOP;
  heading_ = 0;
  setAccel(25);

  headingAvg_.SetWindow(10);
}

void RAMBLERBot::init()
{
  // Wire.begin();
  // compass_.init();
  // compass_.enableDefault();
  
  // // Calibration values. Use the Calibrate example program to get the values for
  // // your compass.
  // // M min X: -644 Y: 49 Z: 1655 M max X: -27 Y: 820 Z: 1772

  // // RAMBLER Uno
  // //compass_.m_min.x = -444; compass_.m_min.y = 282; compass_.m_min.z = 1286;
  // //compass_.m_max.x = -135; compass_.m_max.y = 670; compass_.m_max.z = 1568;
  
  // // RAMBLER Leonardo
  // compass_.m_min.x = 662; compass_.m_min.y = 241; compass_.m_min.z = 1509;
  // compass_.m_max.x = 1024; compass_.m_max.y = 753; compass_.m_max.z = 1726;
  
  //SeedRandomGenerator();
}


void RAMBLERBot::setAccel(int maxAccel)
{
  max_accel_per_loop_ = maxAccel;
}

// FAST LOOP: 50 Hz-ish
void RAMBLERBot::loop50Hz()
{
  // compass_.read();
  // heading_ = compass_.heading((LSM303::vector){0,-1,0})-180;
  // headingAvg_.Push(heading_);
  
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

void RAMBLERBot::loop10HzOffset()
{
  // Maybe read the whisker in here or do other relevant operations??
}

// SLOW LOOP: 10 Hz-ish
void RAMBLERBot::loop10Hz()
{
  int v_fwd, angle, goal_dir;
  int ant_right, ant_left, ant_set;
  int v_omg;
  
  // Read the Antenna every loop
  whiskers_.GetWhiskerValues(&ant_right, &ant_left);
  ant_set = 50; // Constant antenna set point (for now)
  
  // Calculate velocities
  v_fwd = (motor_right_+motor_left_)/2;
  v_omg = (motor_right_-motor_left_); // Say angular velocity is right-left
    
  // Check the Emergency Stop
  if (state_ != ESTOP && button_.isPressed())
  {
    index_ = 0;                         // Reset state machine
    start_ = false;
    state_ = ESTOP;  
    motors_.setRightSpeed(0);           // Stop motors
    motors_.setLeftSpeed(0);
    motor_left_ = 0;
    motor_right_ = 0;
    motor_left_goal_ = 0;
    motor_right_goal_ = 0;
    lights_.AllOff();                   // Turn off lights
    whiskers_.ResetCalibration();       // Reset the whisker
    buzzer_.playNote(NOTE_G(5),100,15); // Play a sound
    button_.waitForRelease();           // wait here for the button to be released
  }
  
  // Progress through the overall Robot States
  switch (state_)
  {    
    case ESTOP:
      if (index_++ >= 5)
      {
        lights_.ToggleLED2();
        index_ = 0;
      }
      motor_left_goal_ = 0;
      motor_right_goal_ = 0;
      if (button_.isPressed())
      {
        // Prepare for the robot to run!!!
        index_ = 0;
        lights_.TurnOffLED2();     // Turn off lights
        state_ = ESTOP_TO_ACTIVE;  // Next State
        SeedRandomGenerator();     // Seed generator
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
      Serial.println(GetRandomValue());
      if (index_ < 10)
      {
        index_++;
      }
      else
      {
        index_ = 0;
        state_ = TEST_WALLFOLLOW;
      }
    break;
      
    case COCKROACH:
      // Todo: Calibrate Velocity!!
      //  -> Calibrated: At full battery charge, motor_left_ and motor_right_ = 250 
      //        makes the robot run approx 3 bodylen/s
      angle = 0;
      goal_dir = 0;
      ant_left = 0;
      ant_right = 0;
      brain.ProcessInput(random(1001),random(1001),v_fwd,angle,ant_right,ant_left,goal_dir);
      brain.GetVelocity(&motor_left_goal_, &motor_right_goal_);
      lights_.DisplayChar(brain.getLights());
    break;
      
    case TEST_FORWARD:
      motor_left_goal_ = 250;
      motor_right_goal_ = 250;
      state_ = TEST_PIVOT;
    break;
    
    case TEST_WALLFOLLOW:
      
      // // WallFollow!
      ant_set = 50; // TODO: Find antenna set point
      v_omg = 1*(ant_set-ant_right)*2;  //Perform wallfollowing control
      v_omg = max(min(v_omg,100),-100);
      v_fwd = max(250 - fabs(v_omg), 100);       // Scale v_fwd based on omega
      motor_right_goal_ = v_fwd + v_omg/2;
      motor_left_goal_ = v_fwd - v_omg/2;
      
      Serial.print("Left: ");
      Serial.print(ant_left);
      Serial.print("Right: ");
      Serial.print(ant_right);
      Serial.print("RightMotor: ");
      Serial.print(motor_right_goal_);
      Serial.print("LeftMotor: ");
      Serial.println(motor_left_goal_);
      
    break;
    
    case TEST_PIVOT:
      if (index_ < 10)
      {
        motor_left_goal_ = 250;
        motor_right_goal_ = 250;
        brain.SetState(RamblerAlgorithm::PIVOT);
        index_++;
      }
      else
      {
        brain.ProcessInput(random(1001),random(1001),v_fwd,0,0.0,0.0,0);
        brain.GetVelocity(&motor_left_goal_, &motor_right_goal_);
        Serial.println("2");
        if (brain.getState() == RamblerAlgorithm::STRAIGHT)
        {
          index_ = 0;
          state_ = TEST_END;
        }
      }
    break;
        
    case TEST_END:
      if (index_ < 5)
      {
        motor_left_goal_ = 250;
        motor_right_goal_ = 250;
        index_++;
      }
      else
      {
        motor_left_goal_ = 0;
        motor_right_goal_ = 0;
        index_ = 0;
        state_ = ESTOP;
      }
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
