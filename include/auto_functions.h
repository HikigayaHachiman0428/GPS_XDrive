#ifndef _AUTO_FUNCTIONS_
#define _AUTO_FUNCTIONS_
#include "definitions_and_declarations.h"
#include "headers.h"
#include "PID.h"
#undef __ARM_NEON__
#undef __ARM_NEON
#include <eigen-3.4.0/Eigen/Dense>

/* Basic Functions */

void moveMotor(motor M, float v)
{
  // v = fmin(abbs(v), 100) * sign(v);
  v *= 120;
  M.spin(fwd, v, vex::voltageUnits::mV);
}

void driveForward(float v)
{
  moveMotor(LF, v);
  moveMotor(LB, v);
  moveMotor(RF, -v);
  moveMotor(RB, -v);
}

void driveHorizontal(float v)
{
  moveMotor(LF, v);
  moveMotor(LB, -v);
  moveMotor(RF, v);
  moveMotor(RB, -v);
}

void driveRotate(float v)
{
  moveMotor(LF, -v);
  moveMotor(LB, -v);
  moveMotor(RB, -v);
  moveMotor(RF, -v);
}

void timerForward(float v, float t)
{
  driveForward(v);
  delay(t);
  driveForward(0);
}

void timerHorizontal(float v, float t)
{
  driveHorizontal(v);
  delay(t);
  driveHorizontal(0);
}

void timerRotate(float v, float t)
{
  driveRotate(v);
  delay(t);
  driveRotate(0);
}

void setStartGyro(float start)
{
  startGyro = start;
  Gyro.setRotation(start, deg);
}

float processTarget(float target)
{
  while (abbs(GYRO - target) > 180)
  {
    if (target < GYRO)
    {
      target += 360;
    }
    else
      target -= 360;
  }
  return target;
}

void PIDGyroTurn(float target, float tolerance = 1, float dTolerance = 1)
{
  auto pid = PID();
  pid.setCoefficient(4.5, 0.2, 15.75);
  pid.setErrorTolerance(tolerance);
  pid.setTarget(processTarget(target));
  pid.setIMin(0);
  pid.setIstart(10);
  pid.setdTolerance(dTolerance);
  while (!pid.targetArrived())
  {
    pid.update(GYRO, Gyro.gyroRate(zaxis, dps) * 0.01);
    // cout<<pid.getVelocity()<<endl;
    driveRotate(pid.getOutput());
    delay(10);
  }
  driveRotate(0);
}

/* 赛季特别方法 */

void intake(float v)
{
  moveMotor(itk, v);
  moveMotor(ind, v);
}

void OLDPIDGyroTurn(float target, float tolerance = 1, float dTolerance = 1)
{
  float dt = 0.01;
  float timeroffset = TIMER;
  float timeused = 0;
  float kp = 4.5;      // 3.0 , 3.95
  float ki = 0.2;      // 28.9 , 28.6
  float kd = 22.75;    // 29.9 , 29.9
  float imin = 0 / ki; // ji fen fan wei
  float istart = 15;   // start to integral
  float dtol = dTolerance;
  float errortolerance = tolerance; // 2.5zd
  float lim = 100;
  target = processTarget(target);
  float error = target - GYRO;
  float lasterror;
  float v = 0;
  float i = 0;
  bool arrived, firstOver = true;
  float timetol = abbs(error) < 20 ? 700 : abbs(error) * 24;
  float pow, slow = 0;
  lasterror = error;
  arrived = error == 0;
  while (!arrived)
  {
    timeused = TIMER - timeroffset;
    error = target - Gyro.rotation();
    // v = (error - lasterror) / dt;
    v = Gyro.gyroRate(zaxis, dps) * dt;
    if ((abbs(error) < errortolerance && abbs(v) <= dtol)
        //||timeused > timetol
    )
    {
      arrived = true;
    }

    if (preOpenLid && abbs(error) < lidStart)
      intake(-100);
    if (abbs(error) < istart)
      i += sign(error) * dt;
    if (error * lasterror <= 0)
    {
      if (firstOver)
      {
        i = sign(error) * imin;
        firstOver = false;
      }
      else
      {
        i = 0;
      }
    }
    pow = kp * error + kd * v + ki * i;
    pow = abbs(pow) > lim ? sign(pow) * lim : pow;
    driveRotate(pow);
    // cout << pow << "   " << error << "    " << v << "   " << endl;
    // printScreen(10,100,"Iner %f",Iner.rotation());
    lasterror = error;
    delay(1000 * dt);
  }
  // cout << timeused << endl;
  driveForward(0);
}

void timerIntake(float v, float t)
{
  intake(v);
  delay(t);
  intake(0);
}

void timerIndex(float v, float t)
{
  moveMotor(ind, v);
  delay(t);
  moveMotor(ind, 0);
}

void encoderIndex(float v, float encoderTarget)
{
  float timerDealine = 1000;
  float timerOffset = TIMER;
  ind.resetPosition();
  while (abbs(ind.position(deg)) < encoderTarget && (TIMER - timerOffset) <= timerDealine)
  {
    moveMotor(ind, v);
  }
  moveMotor(ind, 0);
}

void lockTrigger()
{
  triggerLock.set(0);
}
void releaseTrigger()
{
  triggerLock.set(1);
}
#define triggerLocked (triggerLock.value() == 0)
#define triggerReleased (triggerLock.value() == 1)

void moveTrigger(float v)
{
  moveMotor(triggerLeft, v);
  moveMotor(triggerRight, v);
}

void holdTrigger()
{
  triggerLeft.stop(hold);
  triggerRight.stop(hold);
}

void resetTriggerEncoder()
{
  triggerLeft.resetPosition();
  triggerRight.resetPosition();
}

float getTriggerEncoder() { return triggerLeft.position(deg); }

void encoderMoveTrigger(float v, float encoderTarget)
{
  triggerLeft.resetPosition();
  while (triggerLeft.position(deg) <= encoderTarget)
  {
    moveTrigger(v);
  }
  moveTrigger(0);
}

void openLid()
{
  moveMotor(ind, -100);
  delay(300);
}
void closeLid()
{
  moveMotor(ind, 100);
  delay(200);
  moveMotor(ind, 0);
}

void liftIntake(bool lift)
{
  intakeUplift.set(lift);
}

void expand(bool E)
{
  expansion.set(E);
}

void rageIntake()
{
  driveForward(-30);
  delay(40);
  liftIntake(false);
  driveForward(-100);
  delay(100);
  driveForward(20);
  delay(50);
  driveForward(0);
  delay(500);
  driveForward(50);
  delay(340);
  driveForward(0);
  delay(500);
}

#endif