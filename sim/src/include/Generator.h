#pragma once
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
  static int buildPath(Spline &spline, int removeNodes = 0);
  static double calculateSegLength(int node, Spline spline);
  static void setStepSize(double step) {
    _stepSize = step;
  }
 private:
  static double _stepSize;
};