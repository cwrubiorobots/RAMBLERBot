#ifndef RamblerSerial_h
#define RamblerSerial_h

#include <Arduino.h>

#define LEONARDO 1

// Zumo Serial initializes and handles the RAMBLER serial
// communication to the raspberry pi
class RamblerSerialRcv
{
  public:
    
    // constructor
    RamblerSerialRcv();
    
    enum SerialState {
      HEADER = 0,
      MSG_TYPE,
      MSG
    };
    
    enum MsgType {
      GOAL_THT = 0
    };

    // Empties the Arduino Serial buffer and parses all bytes
    void ParseSerial();
    
    // Passes a single byte through the serial state machine
    void ParseByte(signed char a);
    
    void Reset();
    
    int getGoalDir() { return goal_tht_; }
    
  private:
    SerialState state_;
    
    int goal_tht_;
    int at_goal_;
    
    int msg_count_;
    MsgType type_;
    
    unsigned char msg_header;
};

#endif