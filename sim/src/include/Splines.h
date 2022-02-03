#pragma once

#include <vector>
#include <math.h>

#ifndef M_PI // Sometimes windows is wack and doesn't have M_PI
#define M_PI 3.14159265358979323846264338327
#endif

// Giving progress bar defines. (60 wide)
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

// Print live progress bar using printf() 0-1 value
static void printProgress(double percentage) {
  int val = (int) (percentage * 100);
  int lpad = (int) (percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
  fflush(stdout);
}

struct Waypoint {
  double x, y;
  double segLength = 0; // length of segment to next waypoint
  double totalLength = 0; // length of all segments up to and including this segment
};

// Used as returner for getSplinePoint & getSplinePointGradient
// Uses flag (int) to determin return type, 0 is good, 1 is complete, -1 is error
struct SplinePoint {
  Waypoint waypoint;
  int flag;
};

struct Spline {
  std::vector<Waypoint> waypoints; // waypoints where the robot goes through (the first and last waypoints are ignored in this process [controlpoints])
  double totalLength = 0; // total length of spline (all segments)
  double segmentNum = 0; // number of segments in spline with a value
};

struct CatmullRom {
  static SplinePoint getSplinePoint(float t, Spline spline) {
    int p0, p1, p2, p3;

    p1 = (int)t + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p0 = p1 - 1;

    if (p3 >= spline.waypoints.size()) {
      return {{0,0}, 1}; // out of scope (return segment complete -> 1)
    }

    t = t - (int)t;

    float tt = t * t;
    float ttt = tt * t;

    float q1 = -ttt + 2.0f*tt - t;
    float q2 = 3.0f*ttt - 5.0f*tt + 2.0f;
    float q3 = -3.0f*ttt + 4.0f*tt + t;
    float q4 = ttt - tt;

    float tx = 0.5f * (spline.waypoints[p0].x * q1 + spline.waypoints[p1].x * q2 + spline.waypoints[p2].x * q3 + spline.waypoints[p3].x * q4);
    float ty = 0.5f * (spline.waypoints[p0].y * q1 + spline.waypoints[p1].y * q2 + spline.waypoints[p2].y * q3 + spline.waypoints[p3].y * q4);
    
    // Detect if it's actually a number and isn't INF or NaN
    if (isinf(tx) || isinf(ty) || isnan(tx) || isnan(ty)) {
      return {{0,0}, -1};
    } else {
      return{ tx, ty };
    }
  }

  static SplinePoint getSplineGradientPoint(float t, Spline spline) {
    int p0, p1, p2, p3;
    p1 = (int)t + 1;
    p2 = p1 + 1;
    p3 = p2 + 1;
    p0 = p1 - 1;

    if (p3 >= spline.waypoints.size()) {
      return {{0,0}, 1}; // out of scope (return segment complete -> 1)
    }

    t = t - (int)t;

    float tt = t * t;
    // float ttt = tt * t;

    float q1 = -3.0f * tt + 4.0f*t - 1;
    float q2 = 9.0f*tt - 10.0f*t;
    float q3 = -9.0f*tt + 8.0f*t + 1.0f;
    float q4 = 3.0f*tt - 2.0f*t;

    float tx = 0.5f * (spline.waypoints[p0].x * q1 + spline.waypoints[p1].x * q2 + spline.waypoints[p2].x * q3 + spline.waypoints[p3].x * q4);
    float ty = 0.5f * (spline.waypoints[p0].y * q1 + spline.waypoints[p1].y * q2 + spline.waypoints[p2].y * q3 + spline.waypoints[p3].y * q4);

    // Detect if it's actually a number and isn't INF or NaN
    if (isinf(tx) || isinf(ty) || isnan(tx) || isnan(ty)) {
      return {{0,0}, -1};
    } else {
      return{ tx, ty };
    }
  }

  /**
   * Get angle in radians based from t value and spline
   * returns waypoint and flag
   */
  virtual double getSplineAngleRad(float t, Spline spline) {
    SplinePoint gradientPoint = getSplineGradientPoint(t, spline);
    if (gradientPoint.flag != -1) {
      return atan2(gradientPoint.waypoint.y, gradientPoint.waypoint.x);
    } else {
      return 0;
    }
  }

  /**
   * Get angle in degrees based from t value and spline
   */
  virtual double getSplineAngleDeg(double t, Spline spline) {
    return (getSplineAngleRad(t, spline) * 180 / M_PI); // convert radians to degrees
  }
};