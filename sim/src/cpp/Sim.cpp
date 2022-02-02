#include "Sim.h"

#include "robot/Motor.h"
#include "robot/Encoder.h"
#include "Generator.h"

double rotations = 16/32; // 0.5 rotations per meter

double motors = 0;

Motor m1{0},m2{1},m3{2},m4{3};
Encoder leftEnc{0};
Encoder rightEnc{1};

Spline spline {{
  {0,0},{1,0},{2,0},{3,0}
}};

void initSpline() {
  Generator::buildPath(spline);
}

/**
 * Initializer (Updates once)
 */
void Sim::Init() {
  initSpline();
  std::cout << "Total Length: " << spline.totalLength << std::endl;
}

/**
 * Periodic Update
 */
void Sim::Periodic() {

  // std::cout << "\n" << motors << std::endl;

  m1.set(0.5);
  m3.set(0.5);
  // std::cout << "Encoder: " << leftEnc.getRotations() << std::endl;
}