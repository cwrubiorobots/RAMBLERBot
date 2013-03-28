#include "RamblerSerialRcv.h"

RamblerSerialRcv::RamblerSerialRcv()
{
  state_ = HEADER;
  goal_tht_ = 180; // default goal direction is straight reverse
  at_goal_ = 0;    // default to NOT at the goal
  
  msg_header = 250;
  
  #if LEONARDO
  Serial1.begin(9600);
  Serial.begin(9600);
  #endif
}

void RamblerSerialRcv::ParseSerial()
{
  while (Serial1.available() > 0)
  {
  #if LEONARDO
    ParseByte(Serial1.read());
  #else
    ParseByte(Serial.read());
  #endif
  }
}

void RamblerSerialRcv::ParseByte(signed char a)
{
  #if LEONARDO
  Serial.print("Byte: ");
  Serial.println(a);
  #endif
  switch (state_)
  {
    // HEADER
    // Transitions:
    //   1. Go to MSG_TYPE if header is found
    case HEADER:
      if ((unsigned char)a == msg_header)
      {
        state_ = MSG_TYPE;
        #if LEONARDO
        Serial.println("Header Received");
        #endif
      }
      else
      {
        #if LEONARDO
        Serial.println("Header FAILED");
        #endif
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
        #if LEONARDO
        Serial.print("Message received: ");
        Serial.println(a);
        #endif
      }
      else
      {
        // State is not recognized
        state_ = MSG_TYPE;
        #if LEONARDO
        Serial.println("Message NOT RECOGNIZED");
        #endif
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
          #if LEONARDO
          Serial.print("Goal Tht: ");
          Serial.println(goal_tht_);
          #endif
        }
        else if (msg_count_ == 1)
        {
          at_goal_   = a; // "0" if not at the goal, "1" if at the goal
          msg_count_ = 0;
          state_     = HEADER;
          #if LEONARDO
          Serial.print("At Goal: ");
          Serial.println(at_goal_);
          #endif
        }
      }
      break;
      
    default:
      state_ = HEADER;
      msg_count_ = 0;
      break;
  }
}