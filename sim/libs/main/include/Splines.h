#pragma once

#include <vector>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

struct Waypoint {
  double x, y;
};

struct Spline {
  std::vector<Waypoint> waypoints; // waypoints where the robot goes through
  Waypoint entryCtrl, exitCtrl;
  double totalLength = 0;
  double actualLength = 0;
};

struct CatmullRom {
  static Waypoint getSplinePoint(double t, Spline spline) {
    int p0, p1, p2, p3;

    p1 = (int)t + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p0 = p1 - 1;

    t = t - (int)t;

    double tt = t * t;
    double ttt = tt * t;

    double q1 = -ttt + 2.0f*tt - t;
    double q2 = 3.0f*ttt - 5.0f*tt + 2.0f;
    double q3 = -3.0f*ttt + 4.0f*tt + t;
    double q4 = ttt - tt;

    double tx = 0.5f * (spline.waypoints[p0].x * q1 + spline.waypoints[p1].x * q2 + spline.waypoints[p2].x * q3 + spline.waypoints[p3].x * q4);
    double ty = 0.5f * (spline.waypoints[p0].y * q1 + spline.waypoints[p1].y * q2 + spline.waypoints[p2].y * q3 + spline.waypoints[p3].y * q4);

    return{ tx, ty };
  }

  static Waypoint getSplineGradientPoints(double t, Spline spline) {
    int p0, p1, p2, p3;
    p1 = (int)t + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p0 = p1 - 1;

    t = t - (int)t;

    double tt = t * t;
    // double ttt = tt * t;

    double q1 = -3.0f * tt + 4.0f*t - 1;
    double q2 = 9.0f*tt - 10.0f*t;
    double q3 = -9.0f*tt + 8.0f*t + 1.0f;
    double q4 = 3.0f*tt - 2.0f*t;

    double tx = 0.5f * (spline.waypoints[p0].x * q1 + spline.waypoints[p1].x * q2 + spline.waypoints[p2].x * q3 + spline.waypoints[p3].x * q4);
    double ty = 0.5f * (spline.waypoints[p0].y * q1 + spline.waypoints[p1].y * q2 + spline.waypoints[p2].y * q3 + spline.waypoints[p3].y * q4);

    return { tx, ty };
  }

  static double getAngleRad(double t, Spline spline) {
    Waypoint gradient = getSplineGradientPoints(t, spline);
    return atan2(gradient.y, gradient.x);
  }

  static double getAngleDeg(double t, Spline spline) {
    return (getAngleRad(t, spline) * (180/M_PI));
  }
};