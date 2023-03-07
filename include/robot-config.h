#ifndef _CONFIG_
#define _CONFIG_

#include "headers.h"
// #include "definitions_and_declarations.h"
#include <eigen-3.4.0/Eigen/Dense>
using namespace vex;

brain Brain;

motor LF = motor(PORT2, ratio6_1, 1);
motor LB = motor(PORT1, ratio6_1, 1);
motor RF = motor(PORT9, ratio6_1, 1);
motor RB = motor(PORT8, ratio6_1, 1);
motor itk = motor(PORT17, ratio36_1, 0);
motor ind = motor(PORT11, ratio18_1, 1);
motor triggerLeft = motor(PORT20, ratio36_1, 1);
motor triggerRight = motor(PORT15, ratio36_1, 0);

inertial Gyro = inertial(PORT10, turnType::left);
encoder encoderx = encoder(Brain.ThreeWirePort.A); // A, B
encoder encodery = encoder(Brain.ThreeWirePort.E); // E, F
triport ThreeWireExtension = triport(PORT7);
digital_out triggerLock = digital_out(ThreeWireExtension.B);
digital_out intakeUplift = digital_out(ThreeWireExtension.C);
digital_out expansion = digital_out(ThreeWireExtension.A);

// CoR relative to Omnipos
float distancex = 16.5;
float distancey = 0;
Eigen::Vector2f Omnipos_To_CoR = Eigen::Vector2f(distancex, distancey);
Eigen::Vector2f CoR_To_Intake = Eigen::Vector2f(32, 0) - Omnipos_To_CoR;

#endif