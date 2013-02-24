#ifndef MovingAverage_h
#define MovingAverage_h


#include <Arduino.h>
#include <LList.h>
#include <AngleUtilities.h>

/*
*/

// Implements a moving average using linked lists
// EJ Kreinar- ejk43@case.edu
class MovingAverage 
{
public:
  MovingAverage() : m_window(0), m_n(0) {}
  
  // Clears the Moving Average
  // ... Does not change the window size
  void Clear()
  {
    m_n = 0;
    m_vals.clear();
  }
  /*
  int angleDifference(int a, int b)
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
  }*/
  
  // Resets the window size
  // Also clears the moving average
  void SetWindow(int window)
  {
    if (window > 0)
    {
      m_vals.clear();
      m_n = 0;
      m_window = window;
    }
  }

  // Adds a value to the moving average
  // Deletes an element from the list if necessary
  void Push(double x)
  {
    m_total += x;
    m_vals.push_back(x);  // Add a new element
    if (m_n < m_window)
      m_n++;                // Increment number of elements
    else
      m_total -= m_vals.pop_front();
    m_mean = m_total/static_cast<float>(m_n);
  }

  // Returns the number of data values
  int NumDataValues() const
  {
    return m_n;
  }

  // Calculates the mean of the moving average
  float Mean()
  {
    return m_mean;
  }
  
  // Calculates the mean Angle of the moving average
  // (Necessary when average angles around -180, +180)
  float AngleMean()
  {
    if (m_n == m_window)
    {
      float total = 0;
      int start = m_vals.getElement(0);
      for (int i = 1; i < m_window; i++)
      {
        total += AngleUtilities::angleDifference(m_vals.getElement(i),start); //Todo: Iterate through list itself
      }
      m_mean = (total / static_cast<float>(m_window-1))+start;
    }
    return (m_n == m_window) ? m_mean : 0.0;
  }

private:
  int m_window; // Size of window
  int m_n;      // Current total # of elements in filter
  LList<int> m_vals;
  float m_mean;
  float m_total;
};

#endif