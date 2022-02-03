#include <math.h>
#include <iostream>
#ifdef __linux__ 
  #include <bits/stdc++.h>
#elif _WIN32
  #include <iomanip>
#endif

#include "Splines.h"


class RobotStuff {
 public:
  static SplinePoint locationOnPath(double encoderRotations, double splineLength, Spline spline);
  static double findAngle();

 private:
  
};