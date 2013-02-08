#ifndef AngleUtilities_h
#define AngleUtilities_h

// Provides static member functions for angle operations
// (I tried to include these functions in a namespace 
//  but the compiler just wasnt happy)
// EJ Kreinar: ejk43@case.edu
class AngleUtilities
{
public:

  static int angleDifference(int a, int b)
  {
    int diff;
    a = a % 360 - 180;
    b = b % 360 - 180;
    diff = a - b;
    if (diff > 180)
      diff -= 360;
    if (diff < -180)
      diff += 360;
    return diff; 
  }

};

#endif