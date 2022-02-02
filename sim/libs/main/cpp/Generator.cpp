#include "Generator.h"
#include "Splines.h"

double Generator::calculateSegLength(int node, Spline spline) {

  Waypoint oldPoint, newPoint;
  oldPoint = CatmullRom::getSplinePoint((double)node, spline);

  std::vector<double> buffer;
  for (double t = 0.0; t < 1.0f; t += 0.05f) {
    newPoint = CatmullRom::getSplinePoint((double)node + t, spline);
    double xrt = (newPoint.x - oldPoint.x)*(newPoint.x - oldPoint.x);
    double yrt = (newPoint.y - oldPoint.y)*(newPoint.y - oldPoint.y);
    long double xyrt = xrt + yrt;
    buffer.push_back(sqrt(xyrt));

    oldPoint = newPoint;
  }

  double length_buff = 0;
  for (int i = 0; i < buffer.size(); i++) {
    // std::cout << "Buffer: " << buffer[i] << std::endl;
    length_buff += buffer[i];
    // std::cout << "Length Buffer: " << length_buff << std::endl;
  }

  return length_buff;
}

Spline Generator::buildPath(Spline spline) {
  double totalLength = 0;
  double pointsLength = 0;
  double segLength = 0;
  double segf_l = 0;
  double segl_l = 0;

  Spline internalSpline = spline;

  internalSpline.waypoints.insert(internalSpline.waypoints.begin(), spline.entryCtrl); // add ctrlpnt to beginning
  internalSpline.waypoints.push_back(spline.exitCtrl); // add to the back of array

  for (int i = 0; i < internalSpline.waypoints.size() - 1; i++) {
    segLength = calculateSegLength(i, internalSpline);

    if (i > 0) {
      if (i < internalSpline.waypoints.size()-1)
        pointsLength += segLength;
    }

    totalLength += segLength;

    // std::cout << "Length: " << totalLength << std::endl;
    // std::cout << "Seg Length: " << pointsLength << std::endl;

  }

  // std::cout << "\nFinal Length: " << totalLength << std::endl;

  // Waypoint startPoint = spline.waypoints.front();
  Waypoint endPoint = spline.waypoints.back();

  double actualLength = 0;
  bool foundDistance = false;
  while (foundDistance == false) {
    Waypoint point = CatmullRom::getSplinePoint(actualLength, internalSpline);
    double angle = CatmullRom::getAngleDeg(actualLength, internalSpline);

    // std::cout << std::fixed << std::setprecision(5) << "Distance: " << actualLength << ", Angle: " << angle << ", x,y: (" << point.x << "," << point.y << ")" << std::endl;

    if (point.x < endPoint.x || point.y < endPoint.y) {
      actualLength += _stepSize;
    } else {
      foundDistance = true;
    }
  }
  
  // Spline output = spline;
  internalSpline.totalLength = totalLength;
  internalSpline.actualLength = actualLength;
  return internalSpline;
}

double Generator::getAngle(float splineAngle, float encoderVals) {
  double rightMotorVal = 0;
  double leftMotorVal = 0;

  // Spline internalSpline = spline;

  std::cout << splineAngle << std::endl;
  std::cout << encoderVals << std::endl;

  if (encoderVals < splineAngle) {
    rightMotorVal = 1;
    leftMotorVal = 0;
  } else if (encoderVals < splineAngle) {
    rightMotorVal = 0;
    leftMotorVal = 1;
  } else {
    rightMotorVal = 1;
    leftMotorVal = 1;
  }

  return rightMotorVal, leftMotorVal;
}