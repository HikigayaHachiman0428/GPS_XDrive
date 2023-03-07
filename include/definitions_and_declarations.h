#ifndef DEFINITIONS_H
#define DEFINITIONS_H
#undef __ARM_NEON__
#undef __ARM_NEON
#include <iostream>
#include "headers.h"
#include "robot-config.h"
#include <eigen-3.4.0/Eigen/Dense>
using namespace std;
using namespace vex;
#define TIMER Brain.timer(vex::timeUnits::msec)
controller Controller1 = controller(primary);

#define Ch1 Controller1.Axis1.position(percent)
#define Ch2 Controller1.Axis2.position(percent)
#define Ch3 Controller1.Axis3.position(percent)
#define Ch4 Controller1.Axis4.position(percent)

#define BA Controller1.ButtonA.pressing()
#define BB Controller1.ButtonB.pressing()
#define BX Controller1.ButtonX.pressing()
#define BY Controller1.ButtonY.pressing()

bool L1()
{
    return Controller1.ButtonL1.pressing();
}
bool L2() { return Controller1.ButtonL2.pressing(); }
bool R1() { return Controller1.ButtonR1.pressing(); }
bool R2() { return Controller1.ButtonR2.pressing(); }

#define UP Controller1.ButtonUp.pressing()
#define DOWN Controller1.ButtonDown.pressing()
#define LEFT Controller1.ButtonLeft.pressing()
#define RIGHT Controller1.ButtonRight.pressing()

#define cPrint(x)                \
    Controller1.Screen.print(x); \
    Controller1.Screen.newLine();
#define bPrint(x, y, s) Brain.Screen.printAt(x, y, s)

float sign(float x)
{
    return (x == 0) ? 0 : (x > 0 ? 1 : -1);
}
float abbs(float x)
{
    return x >= 0 ? x : -x;
}

#define delay vexDelay
#define GYRO Gyro.rotation(deg)
#define V2 Eigen::Vector2f
#define V4 Eigen::Vector4f
#define lidDis 10
static int ch_state = 1;
static int autoRoutine = 0;
static bool lockMode = 0;
static bool locked = 0;
static bool adjustTrigger = 1;
static bool triggerReady = 0;
static Eigen::Vector2f shootTarget;
static float triggerDisTarget;
static float startGyro;
static bool autoTrigger = true;
static bool inOneMinute = false;
static bool preOpenLid = false;
static float lidStart = 0;
static int discNum = 0;
static bool intakeControlOverriden = false;
#endif