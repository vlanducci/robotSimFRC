#include "Sim.h"

#include "robot/Motor.h"
#include "robot/Encoder.h"
#include "Generator.h"
#include "RobotStuff.h"
#include "Splines.h"

double rotations = 16/32; // 0.5 rotations per meter

double motors = 0;

Motor m1{0}, m2{1}, m3{2}, m4{3};
Encoder leftEnc{0};
Encoder rightEnc{1};

Spline spline {{
  {0,0},{1,0},{2,0},{3,0}
}};

int output = 0;
SplinePoint locationOnPath = {0,0};
double leftEncVal = 0;
double rightEncVal = 0;
double avgEncVal = 0;
double totalLength = 0;

/**
 * Initializer (Updates once)
 */
void Sim::Init() {
  output = Generator::buildPath(spline);
  std::cout << "Total Length: " << spline.totalLength << std::endl;
}

/**
 * Periodic Update
 */
void Sim::Periodic() {
  leftEncVal = leftEnc.getRotations();
  rightEncVal = rightEnc.getRotations();
  avgEncVal = (leftEncVal + rightEncVal) / 2;
  totalLength = spline.totalLength;

  locationOnPath = RobotStuff::locationOnPath(avgEncVal, totalLength, spline);
  m1.set(0.5);
  m3.set(0.5);
  // std::cout << "Encoder Rotations: " << leftEnc.getRotations() << std::endl;
}