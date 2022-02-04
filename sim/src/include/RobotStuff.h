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
  static float tValue(double encoderRotations, double splineLength);
  static SplinePoint locationOnPath(float t, Spline spline);
  static double followSpline(float t, Spline spline);

 private:
  float t = 0.0;
  double totalRotations = 0;
};