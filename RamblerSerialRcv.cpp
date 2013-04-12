#include "RamblerSerialRcv.h"

RamblerSerialRcv::RamblerSerialRcv()
{
  state_ = HEADER;
  goal_tht_ = 1000; // default goal direction is straight reverse
  at_goal_ = 0;    // default to NOT at the goal
  
  msg_header = 250;
  
  #if LEONARDO
  Serial1.begin(9600);
  Serial.begin(9600);
  #endif
  
  parse_count_ = 0;
}

void RamblerSerialRcv::ParseSerial()
{
  // parse_count_ tracks the # of times that ParseSerial has been called
  // If ParseSerial is called 20 times (2 seconds) without finding a new message, 
  // then revert to the default goal_tht_
  if (parse_count_++ >= 20)
  {
    goal_tht_ = 1000;
    parse_count_ = 0;
  }
  
  #if LEONARDO
  while (Serial1.available() > 0)
  {
    ParseByte(Serial1.read());
  }
  #else
  while (Serial.available() > 0)
  {
    ParseByte(Serial.read());
  }
  #endif
}

void RamblerSerialRcv::ParseByte(signed char a)
{
  switch (state_)
  {
    // HEADER
    // Transitions:
    //   1. Go to MSG_TYPE if header is found
    case HEADER:
      if ((unsigned char)a == msg_header)
      {
        state_ = MSG_TYPE;
      }
      else
      {
      }
      break;
  
    // MSG_TYPE:
    // Transitions:
    //   1. Go to MSG if the type is valid
    //   2. Go back to header if the message type is not valid
    case MSG_TYPE:;
      if (a == (unsigned char)GOAL_THT)
      {
        state_ = MSG;
        type_  = GOAL_THT;
        msg_count_ = 0;
      }
      else
      {
        // State is not recognized
        state_ = MSG_TYPE;
      }
      break;
    
    // MSG:
    // Transitions:
    //   1. Go to HEADER once the msg is done
    case MSG:
      if (type_ == GOAL_THT)
      {
        if (msg_count_ == 0)
        {
          goal_tht_ = 2*a; // Goal angle is twice the received value 
          msg_count_++;
          parse_count_ = 0;
        }
        else if (msg_count_ == 1)
        {
          at_goal_   = a; // "0" if not at the goal, "1" if at the goal
          msg_count_ = 0;
          state_     = HEADER;
        }
      }
      break;
      
    default:
      state_ = HEADER;
      msg_count_ = 0;
      break;
  }
}