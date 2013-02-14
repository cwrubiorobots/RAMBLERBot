#ifndef RamblerAlgorithm_h
#define RamblerAlgorithm_h

#include <Arduino.h>
#include <LList.h>

/*
namespace AlgorithmState
{
enum state {
  STRAIGHT = 0,
  PIVOT
};
}*/

#define RAMBLER_HZ 10

// RamblerAlgorithm:
// Implements the actual Rambler Algorithm
class RamblerAlgorithm
{
  public:
    
    // constructor
    // TODO: Maybe provide a parameter for loop rate??
    RamblerAlgorithm();
    
    // Enum for tracking the cockroach state
    enum RamblerState {
      STRAIGHT = 0,
      PIVOT
    };
    
    // algorithm_state_t Process(..)
    // Given the sensor inputs, determine the next robot state
    // Inputs: 
    //    - int rand: Random integer between 0 and 1000
    //    - int angle: Approximate robot heading, -180 to 180 in deg.
    //    - float ant_rgt: Antenna value, 0-5V reading from potentiometer
    //    - float ant_lft: Antenna value, 0-5V reading from potentiometer
    //    - int goal_dir: Direction to goal
    // Returns:
    //    - algorithm_state_t: New robot state
    RamblerState ProcessInput(int rand1, int rand2, int angle, float ant_rgt, float ant_lft, int goal_dir);
    
    // void GetVelocity(float *v, float *w)
    // Populates the desired robot velocities, v and w
    //   - float *v: Populated with desired velocity
    //   - float *w: Populated with desired angular velocity
    void GetVelocity(int *v1, int *v2);
    
    // void SetState(algorithm_state_t)
    // Sets the robot state... could be good for debugging purposes. 
    // Not necessary during normal operation since the state is set
    // when Process() is called
    //  - algorithm_state_t state: New state to set
    void SetState(RamblerState state) {state_ = state;} 
    
    RamblerState getState() {return state_;}
    
    
  private:
    //Member Functions
    
    // Helper classes and variables
    
    // Member variables
    RamblerState state_;
    float dt_;
    float v_; 
    float w_;
    float left_motor_;
    float right_motor_;
    int pivot_dir_;
    int pivot_rate_;
    char pivot_count_;
    
  
};

#endif
