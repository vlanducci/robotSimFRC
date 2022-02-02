#include <math.h>
#include <iostream>
#ifdef __linux__ 
  #include <bits/stdc++.h>
#elif _WIN32
  #include <iomanip>
#endif

#include "Splines.h"

class Generator {
 public:
  static Spline buildPath(Spline spline);
  static double calculateSegLength(int node, Spline spline);
  static double getAngle(float splineAngle, float encoderVals);

  double splineAngle = 0;
 private:
  static constexpr double _stepSize = 0.001;  // change this later!!
};