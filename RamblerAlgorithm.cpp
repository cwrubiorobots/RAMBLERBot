#include <RamblerAlgorithm.h>


RamblerAlgorithm::RamblerAlgorithm()
{
  state_ = RamblerAlgorithm::STRAIGHT;
  dt_ = 1.0/ RAMBLER_HZ;
  v_ = 0.0;
  w_ = 0.0;
  pivot_dir_ = 1;
  
}

RamblerAlgorithm::RamblerState RamblerAlgorithm::ProcessInput(
                    int rand1, int rand2,
                    int angle, 
                    float ant_rgt, 
                    float ant_lft, 
                    int goal_dir )
{
  // Rambler Algorithm implemented as a state machine
  // See Kati Daltorio's RAMBLER Algorithm
  
  // 1. STATE MACHINE TRANSITION
  switch (state_)
  {
  
    //STRAIGHT
    //  Transitions:
    //  1. Go to PIVOT depending on pivot rate & random value
    //  2. Go to WALL_FOLLOW if wall is detected
    //  3. Go to TO_GOAL if goal direction is close to straight
    case RamblerAlgorithm::STRAIGHT:
      // Check for wall:
      if (false)//wall_detected)
      {
        //state_ = RamblerAlgorithm::WALL_FOLLOW;
      }
      else
      {
        // Check for pivot transition: 
        // Pivot rate: (pivot const) * (straight velocity) * (dt)
        // straight velocity = 3 bodylen / s
        // pivot const = .575 pivots / bodylen <--- If goal is not in sight        
        float pivot_rate = 3 * .575 / RAMBLER_HZ;
        if (rand1 < pivot_rate)
        {
          pivot_count_ = 0;
          state_ = RamblerAlgorithm::PIVOT;
          // Continue Direction Ratio
          if (rand2 < 666)
          {
            pivot_dir_ = -1 * pivot_dir_;
          }
        }
      }
    break;
    
    //PIVOT
    //  Transitions:
    //  1. Go to STRAIGHT when pivot done
    //    What happens if we hit a wall during a pivot??
    case RamblerAlgorithm::PIVOT:
      if (rand1 < pivot_count_ * 100)
      {
        pivot_count_ = 0;
        state_ = RamblerAlgorithm::PIVOT;
      }
      else
      {
        pivot_count_++;
      }
    break;
  }
  
  return state_;
}

void RamblerAlgorithm::GetVelocity(int *v1, int *v2)
{
  // 2. DETERMINE OUTPUTS (velocity and angular velocity)
  switch (state_)
  {
  
    //STRAIGHT
    //  Action:
    //    - Set v = 3 bodylen/s and w = 0
    case RamblerAlgorithm::STRAIGHT:
      left_motor_ = 200;
      right_motor_ = 200;
      v_ = (left_motor_ + right_motor_)/2;
      w_ = 0;
    break;
    
    //PIVOT
    //  Action:
    //    - Set w = something and v = 0 
    case RamblerAlgorithm::PIVOT:
      left_motor_ = -1 * pivot_dir_ * 200;
      right_motor_ = pivot_dir_ * 200;
      v_ = 0;
      w_ = (right_motor_ - left_motor_)/0.1;
    break;
  }
  
  *v1 = left_motor_;
  *v2 = right_motor_;
}


