#include <RamblerAlgorithm.h>

int RangeLimit(int in, int min_in, int max_in)
{
  return max(min(in,max_in),min_in);
}

template <typename type>
type sign(type value) { 
  return type((value>0)-(value<0)); 
}

RamblerAlgorithm::RamblerAlgorithm()
{
  state_ = RamblerAlgorithm::STRAIGHT;
  dt_ = 1.0/ RAMBLER_HZ;
  v_ = 0.0;
  w_ = 0.0;
  
  pivot_dir_ = 1;
  wall_dir_ = 1;
  
  ant_set_ = 75;
  ant_P_ = 2;
  
  light_count_ = 0;
  
  ant_right_ = 0;
  ant_left_ = 0;

  loop_count_  = 0;
  
  wall_following_ = false;
}

RamblerAlgorithm::RamblerState RamblerAlgorithm::ProcessInput(
                    int rand1, int rand2, int rand3,
                    int v_fwd,
                    int angle,
                    int ant_rgt,
                    int ant_lft,
                    int goal_dir )
{
  // Rambler Algorithm implemented as a state machine
  // See Kati Daltorio's RAMBLER Algorithm

  // v_fwd varies from 0-250 where 250 is approximately 3 bodylengths per second
  float v_fwd_norm = v_fwd / 100;  // Now v_fwd_norm is in units: bodylen/s
  float exit_rate;
  int compare;
  float dir_ratio;
  
  // Store Antenna Values
  ant_right_ = ant_rgt;
  ant_left_ = ant_lft;
  
  // 1. STATE MACHINE TRANSITION
  switch (state_)
  {
    //STRAIGHT
    //  Transitions:
    //  1. Go to WALL_FOLLOW if wall is detected
    //  2. Go to TO_GOAL if goal direction is close to straight
    //  3. Go to PIVOT depending on pivot rate & random value
    case RamblerAlgorithm::STRAIGHT:
      // Check for wall:
      if (ant_rgt > 0 || ant_lft > 0)//wall_detected)
      {
        // Set the state to the wallfollow
        wall_following_ = false;
        state_ = RamblerAlgorithm::WALL_FOLLOW;
        // Determine the wall direction
        if (ant_rgt > ant_lft)
          wall_dir_ = -1;  // Wall is to the right
        else
          wall_dir_ = 1;   // Wall is to the left
        break;
      }
      
      // Check for Goal Exit
      if (fabs(goal_dir) < 20 && rand3 < 120)
      {
        state_ = RamblerAlgorithm::TO_GOAL;
        break;
      }
      
      // Check for pivot transition: 
      // Pivot rate: (fwd velocity) * (pivot const) * (dt)
      exit_rate = .600;//.575; // Increased to make the robot pivot more often... oops!
      if (fabs(goal_dir) < 15) //10) // Lower the pivot rate if the goal is within 10 degrees
        exit_rate = 0.285;
      if (rand1 < v_fwd_norm * exit_rate / RAMBLER_HZ * 1000)
      {
        state_ = RamblerAlgorithm::PIVOT;
        // Continue Direction Ratio
        dir_ratio = 0.66;
        if (fabs(goal_dir) <= 1) // Is goal detected?
        {
          if (fabs(goal_dir) < 33)   // Check if the goal is with 30 degrees
            dir_ratio = 0.41;//0.51;        // If so, lower the continue direction ratio
          if (sign(goal_dir) * pivot_dir_ == 1) // Check if the previous pivot direction is towards shelter
            dir_ratio = dir_ratio + 0.2;//0.15;       // If so, increase the continue direction ratio by 15% (approximate metric)
        }
        if (rand2 > dir_ratio * 1000)
        {
          pivot_dir_ = -1 * pivot_dir_;
        }
      }
    break;
    
    //PIVOT
    //  Transitions:
    //  1. Go to WALL_FOLLOW if we hit a wall
    //  2. Go to STRAIGHT when pivot done
    case RamblerAlgorithm::PIVOT:
      // Check for wall:
      if (ant_rgt > 0 || ant_lft > 0)//wall_detected)
      {
        // Set the state to the wallfollow
        loop_count_ = 0;
        wall_following_ = false;
        state_ = RamblerAlgorithm::WALL_FOLLOW;
        // Determine the wall direction
        if (ant_rgt > ant_lft)
          wall_dir_ = -1;  // Wall is to the right
        else
          wall_dir_ = 1;   // Wall is to the left
        break;
      }
      
      // Pivot procedure approximates a cockroach-like pivoting PDF
      compare = 1000;
      // Other Options for constants: 
      //   3, 30, 80:  Max around 3-5 timesteps
      //   6, 20, 90:  Max around 4-5 timesteps
      //   7, 15, 75:  Max around 5-6 timesteps
      if (loop_count_ <= 6)
        compare = 20*loop_count_*loop_count_;
      else
        compare = 90*loop_count_;
      if (rand1 < compare)
      {
        loop_count_ = 0;
        state_ = RamblerAlgorithm::STRAIGHT;
        break;
      }
      loop_count_++;
    break;
    
    
    //WALL_FOLLOW (follows a wall)
    //  Transitions:
    //  1. Go to CORNER if a second wall is detected
    //  2. Go to LOST_WALL if the wall is no longer detected
    //  3. Go to WALL_DEPART depending on depart rate and random value
    //  4. Go to WALL_TURN depending on depart rate and random value
    case RamblerAlgorithm::WALL_FOLLOW:
      if (wall_following_ == false) // Make sure we're following with one antenna
      {
        if ((wall_dir_ == 1 && ant_rgt <= 0) || (wall_dir_ == -1 && ant_lft <= 0))
          wall_following_ = true; // Only enter here once we've converged on the wall
      }
      else
      {
        // Check for CORNER:
        if ((wall_dir_ == 1 && ant_rgt > 0) || (wall_dir_ == -1 && ant_lft > 0))
        {
          wall_following_ = false;
          state_ = RamblerAlgorithm::CORNER;
        }
      }
      
      // Check for Lost Wall:
      if ( (wall_dir_ == 1 && ant_lft <= 0) || (wall_dir_ == -1 && ant_rgt <= 0))
      {
        wall_following_ = false;
        state_ = RamblerAlgorithm::LOST_WALL;
        break;
      }
      
      // Check for Depart Wall:
      // Departure rate: (fwd velocity) * (departure const) * (dt)
      exit_rate = 0.155;
      if (fabs(goal_dir) <= 180) // Is goal detected?
      {
        if (sign(goal_dir) * wall_dir_ == 1) // The shelter is behind the wall
        {
          if (fabs(goal_dir) < 90)//112)
            exit_rate = 0.045;
          else
            exit_rate = 0.11;
        }
        else // The shelter is NOT behind the wall
        {
          if (fabs(goal_dir) < 45)
            exit_rate = 0.085;
          else if (fabs(goal_dir) < 90)
            exit_rate = 0.070;
          else if (fabs(goal_dir) < 135)
            exit_rate = 0.070;
          // Otherwise, use the default exit rate of 0.155          
        }
      }
      if (rand1 < v_fwd_norm * exit_rate * 2 / RAMBLER_HZ * 1000) // add a multiply by 1.5 to make it exit more. Or something.
      {
        wall_following_ = false;
        state_ = RamblerAlgorithm::WALL_DEPART;
        break;
      }
      
      // Check for Turnaround Wall:
      // Turnaround rate: (fwd velocity) * (turnaround const) * (dt)
      exit_rate = .032; // Set based on shelter direction
      if (rand2 < v_fwd_norm * exit_rate / RAMBLER_HZ * 1000)
      {
        wall_following_ = false;
        state_ = RamblerAlgorithm::WALL_TURN;
        break;
      }
    break;
    
    //WALL_DEPART
    //  Transitions:
    //  1. Go to WALL_FOLLOW if we hit a wall (on the opposite antenna)
    //  2. Go to STRAIGHT when pivot done
    case RamblerAlgorithm::WALL_DEPART:
      // Check for New Wall:
      if ((wall_dir_ == 1 && ant_rgt > 0) || (wall_dir_ == -1 && ant_lft > 0))
      {
        // Set the state to the wallfollow
        loop_count_ = 0;
        wall_following_ = false;
        state_ = RamblerAlgorithm::WALL_FOLLOW;
        // Determine the wall direction
        if (ant_rgt > ant_lft)
          wall_dir_ = -1;  // Wall is to the right
        else
          wall_dir_ = 1;   // Wall is to the left
        break;
      }
      
      // Depart procedure approximates a cockroach-like pivoting PDF
      compare = 1000;
      if (loop_count_ <= 3)
        compare = 30*loop_count_*loop_count_;
      else
        compare = 80*loop_count_;
      if (rand1 < compare)
      {
        loop_count_ = 0;
        state_ = RamblerAlgorithm::STRAIGHT;
        break;
      }
      loop_count_++;
    break;
    
    //WALL_TURN
    //  Transitions:
    //  1. Go to WALL_FOLLOW if we hit a wall (on the opposite antenna)
    //  2. Go to STRAIGHT if we dont hit a wall within a few timesteps
    case RamblerAlgorithm::WALL_TURN:
      // Check for New Wall:
      if ((wall_dir_ == 1 && ant_rgt > 0) || (wall_dir_ == -1 && ant_lft > 0))
      {
        // Set the state to the wallfollow
        loop_count_ = 0;
        wall_following_ = false;
        state_ = RamblerAlgorithm::WALL_FOLLOW;
        // Determine the wall direction
        if (ant_rgt > ant_lft)
          wall_dir_ = -1;  // Wall is to the right
        else
          wall_dir_ = 1;   // Wall is to the left
        break;
      }
      
      // Time check if the robot doesnt see a new wall
      if (loop_count_ > 15)
      {
        loop_count_ = 0;
        state_ = RamblerAlgorithm::STRAIGHT;
        break;
      }
      loop_count_++;
    break;
    
    
    //LOST_WALL
    //  Transitions:
    //  1. Go to WALL_FOLLOW if we hit a wall
    //  2. Go to STRAIGHT if we've traveled > 1.6 bodylengths
    //  3. Go to AWAY if we dont hit a wall after a few iterations
    case RamblerAlgorithm::LOST_WALL:
      // Check for wall:
      if (ant_rgt > 0 || ant_lft > 0)
      {
        // Set the state to the wallfollow
        loop_count_ = 0;
        wall_following_ = false;
        state_ = RamblerAlgorithm::WALL_FOLLOW;
        // Determine the wall direction
        if (ant_rgt > ant_lft)
          wall_dir_ = -1;  // Wall is to the right
        else
          wall_dir_ = 1;   // Wall is to the left
        break;
      }
      
      compare = 50;
      if (loop_count_ > 20)
      {
        loop_count_ = 0;
        state_ = RamblerAlgorithm::STRAIGHT;
      }
      // else if (rand1 < compare)
      // {
        // // ??? What is this away state??
        // loop_count_ = 0;
        // // state_ = RamblerAlgorithm::AWAY; 
      // }
      else
      {
        loop_count_++;
      }
    break;
    
    //CORNER
    //  Transitions:
    //  1. After turning, decide whether to continue or turnaround
    case RamblerAlgorithm::CORNER:      
      // Run through a set number of iterations
      if (loop_count_ > 10)
      {
        loop_count_ = 0;
        // Transition to Turnaround 20%, Wallfollow 80%
        if (rand1 < 200)
          state_ = RamblerAlgorithm::WALL_TURN;
        else
          state_ = RamblerAlgorithm::WALL_FOLLOW;
        break;
      }
      loop_count_++;
    break;
    
    //TO GOAL
    //  Transitions:
    //  1. When the antenna touches
    case RamblerAlgorithm::TO_GOAL:
      // Check for wall:
      if (ant_rgt > 0 || ant_lft > 0)//wall_detected)
      {
        // Set the state to the wallfollow
        wall_following_ = false;
        state_ = RamblerAlgorithm::WALL_FOLLOW;
        // Determine the wall direction
        if (ant_rgt > ant_lft)
          wall_dir_ = -1;  // Wall is to the right
        else
          wall_dir_ = 1;   // Wall is to the left
        break;
      }
    break;
    
    default:
      state_ = STRAIGHT;
    break;
  }
  
  return state_;
}

void RamblerAlgorithm::GetVelocity(int *vleft, int *vright)
{
  int ant;
  
  // 2. DETERMINE OUTPUTS (velocity and angular velocity)
  switch (state_)
  {
  
    //STRAIGHT
    //  Action:
    //    - Set v = 3 bodylen/s and w = 0
    case RamblerAlgorithm::STRAIGHT:
      left_motor_ = 250; //200
      right_motor_ = 250; //200
      v_ = (left_motor_ + right_motor_)/2;
      w_ = 0;
    break;
    
    //PIVOT
    //  Action:
    //    - Set w = something and v = 0 
    case RamblerAlgorithm::PIVOT:
      left_motor_ = -1 * pivot_dir_ * 80; //200
      right_motor_ = pivot_dir_ * 80; //200
      v_ = 0;
      w_ = (right_motor_ - left_motor_)/0.1;
    break;
    
    //WALL_FOLLOW
    //  Action:
    //    - Set w = Control based on Antenna, v = Something based on w
    case RamblerAlgorithm::WALL_FOLLOW:
      ant = 0;
      // Set the Antenna based on wall direction
      if (wall_dir_ == 1)
        ant = ant_left_;
      else if (wall_dir_ == -1)
        ant = ant_right_;
        
      // Todo: Handle the "headon" case
      if (wall_following_)
      {
        // Wall following steady state (only one antenna senses the wall)
        w_ = wall_dir_ * (ant_set_ - ant) * ant_P_;  //Perform wallfollowing control
        w_ = RangeLimit(w_, -100, 100);              // Limit angular velocity
        v_ = RangeLimit(150 - fabs(w_), 100, 150); // Scale v_fwd based on omega
      }
      else
      {
        // Wall following when both antennas sense the wall...
        w_ = wall_dir_ * 300;  //Perform wallfollowing control
        v_ = -10;
      }
      right_motor_ = v_ + w_/2;
      left_motor_  = v_ - w_/2;
    break;
    
    //WALL_DEPART
    //  Action:
    //    - Set w = something and v = 0 
    case RamblerAlgorithm::WALL_DEPART:
      // Depart acts like a pivot, but turns AWAY from the wall
      left_motor_ = wall_dir_ * 80; //200
      right_motor_ = -1 * wall_dir_ * 80; //200
      v_ = 0;
      w_ = (right_motor_ - left_motor_)/0.1;
    break;
    
    //WALL_TURN
    //  Action:
    //    - Set w = something and v = 0 
    case RamblerAlgorithm::WALL_TURN:
      // Wall Turn turns away from the wall
      left_motor_ = wall_dir_ * 150; //200
      right_motor_ = -1 * wall_dir_ * 150; //200
      v_ = 0;
      w_ = (right_motor_ - left_motor_)/0.1;
    break;
    
    //LOST_WALL
    //  Action:
    //    - Set v = .75 bodylen/s, w= something
    case RamblerAlgorithm::LOST_WALL:
      left_motor_ = -1 * wall_dir_ * 75 + 100;
      right_motor_ = wall_dir_ * 75 + 100;
      v_ = 0;
      w_ = (right_motor_ - left_motor_)/0.1;
    break;
    
    //CORNER
    //  Action:
    //    - Set w = something and v = 0 
    case RamblerAlgorithm::CORNER:
      // Wall Turn turns away from the wall
      left_motor_ = wall_dir_ * 100; //200
      right_motor_ = -1 * wall_dir_ * 100; //200
      v_ = 0;
      w_ = (right_motor_ - left_motor_)/0.1;
    break;
    
    //TO_GOAL
    //  Action:
    //    - Set v = 6 bodylen/s and w = 0
    case RamblerAlgorithm::TO_GOAL:
      left_motor_ = 400;
      right_motor_ = 400;
      v_ = (left_motor_ + right_motor_)/2;
      w_ = 0;
    break;
  }
  
  *vright = right_motor_;
  *vleft = left_motor_;
}

unsigned char RamblerAlgorithm::getLights()
{
  switch (state_)
  {
    //STRAIGHT
    //  Lights
    //    - Middle light solid on
    case RamblerAlgorithm::STRAIGHT:
      return 2;
    break;
    
    //PIVOT
    //  Lights:
    //    - Right light if turning right
    //    - Left light if turning left
    case RamblerAlgorithm::PIVOT:
      if (pivot_dir_ > 0)
        return 1;  
      else
        return 4;
    break;
    
    //WALL_FOLLOW
    //  Lights:
    //    - Right light blinks if following wall on right
    //    - Left light blinks if following wall on left
    case RamblerAlgorithm::WALL_FOLLOW:
      if (light_count_++ > 3)
        light_count_ = 0;
      if (light_count_ > 1)
      {
        if (wall_following_)
        {
          if (wall_dir_ > 0)
            return 1;  
          else
            return 4;
        }
        else
        {
          if (wall_dir_ > 0)
            return 3;  
          else
            return 6;
        }
      }
      else
      {
        return 0;
      }
    break;
    
    //WALL_DEPART
    //  Lights:
    //    - Right light and middle if departing right
    //    - Left light and middle if departing left
    case RamblerAlgorithm::WALL_DEPART:
      if (wall_dir_ > 0)
        return 6;
      else
        return 3;
    break;
    
    //WALL_TURN
    //  Lights:
    //    - Both Left and Right Lights
    case RamblerAlgorithm::WALL_TURN:
      return 5;
    break;
    
    //CORNER
    //  Lights:
    //    - Both Left and Right Lights Blinking
    case RamblerAlgorithm::CORNER:
      if (++light_count_ > 1)
        light_count_ = 0;
      if (light_count_ > 0)
        return 5;
      else
        return 0;
    break;
        
    //TO_GOAL
    //  Lights:
    //    - Middle Light Blinking
    case RamblerAlgorithm::TO_GOAL:
      if (++light_count_ > 1)
        light_count_ = 0;
      if (light_count_ > 0)
        return 2;
      else
        return 0;
    break;
    
    default:
      return 7;
    break;
  }
}

