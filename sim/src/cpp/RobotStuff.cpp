#include "RobotStuff.h"
#include "Splines.h"

float RobotStuff::tValue(double encoderRotations, double splineLength) {
  double meters = encoderRotations / splineLength;
  float t = meters / 16;  // encoder meter val/spline length
  
  std::cout << "\nrotations: " << encoderRotations << std::endl;
  std::cout << "meters: " << meters << std::endl;
  std::cout << "t: " << t << std::endl;

  return t;
}

SplinePoint RobotStuff::locationOnPath(float t, Spline spline) {
  SplinePoint robotCoords = CatmullRom::getSplinePoint(t, spline);

  return robotCoords;
}

double RobotStuff::followSpline(float t, Spline spline) {
  double motorSpeeds [2];

  SplinePoint angle = CatmullRom::getSplineGradientPoint(t, spline);
   
  std::cout << "angle x: " << angle.waypoint.x << std::endl;
  std::cout << "angle y: " << angle.waypoint.y << std::endl;
  
  return 0;
}