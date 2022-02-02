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
  {2,1},
  {3,0}},

  {1,0}, {4,1}
};

Spline outputSpline;

/**
 * Initializer (Updates once)
 */
void Sim::Init() {

}

/**
 * Periodic Update
 */
void Sim::Periodic() {
  // outputSpline = Generator::buildPath(spline);
  motors = Generator::getAngle(4, 1);
  std::cout << "\n" << motors << std::endl;

  m1.set(0.5);
  m3.set(0.5);
  // std::cout << "Encoder: " << leftEnc.getRotations() << std::endl;
}