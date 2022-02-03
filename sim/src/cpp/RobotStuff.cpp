#include "RobotStuff.h"
#include "Splines.h"

SplinePoint RobotStuff::locationOnPath(double encoderRotations, double splineLength, Spline spline) {
  double meters = encoderRotations / 16;
  float t = meters / splineLength;  // encoder meter val/spline length

  std::cout << "Rotations: " << encoderRotations << ", Meters: " << meters << ", t: " << t << std::endl;

  SplinePoint robotCoords;
  robotCoords = CatmullRom::getSplinePoint(t, spline);

  return robotCoords;
}