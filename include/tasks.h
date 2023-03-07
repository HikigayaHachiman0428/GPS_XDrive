#ifndef _TASKS_
#define _TASKS_
#include "definitions_and_declarations.h"
#include "headers.h"
#include "robot-config.h"
#include "auto_functions.h"
#include "GPSfunction.h"

using namespace Eigen;

float GyroValue;
float readGyro() { return GyroValue; }
int GyroReader()
{
  while (1)
  {
    float temp = GYRO;
    while (temp > 180)
      temp -= 360;
    while (temp < -180)
      temp += 360;
    GyroValue = temp;
    delay(10);
  }
}
// 1444.4 122.582

float getDistance(Eigen::Vector2f target)
{
  float predis = sqrt((target[0] - CoR[0]) * (target[0] - CoR[0]) + (target[1] - CoR[1]) * (target[1] - CoR[1]));
  Eigen::Vector2f predicttarget = target - globalSpeed * (16.3 + predis * 0.1);
  // predicttarget << target[0]-globalspeed[0]*(8.3+predis/10), target[1]-globalspeed[1]*(8+predis/8.2);
  float dis = sqrt((predicttarget[0] - CoR[0]) * (predicttarget[0] - CoR[0]) + (predicttarget[1] - CoR[1]) * (predicttarget[1] - CoR[1]));
  return predis;
}

int discNumControl()
{
  float disc1 = 10; // when there is one disc
  float disc2 = 20; // when there are two disc
  float disc3 = 30; // when there are three disc
  float stableLidDis = 0;
  float preLidDis = 0;
  bool preState = false;
  float timerOffset = 0;
  while (1)
  {
    if (preLidDis == lidDis)
      stableLidDis = lidDis;
    if (stableLidDis <= disc1)
      discNum = 0;
    else if (stableLidDis >= disc1 && stableLidDis <= disc2)
      discNum = 1;
    else if (stableLidDis >= disc2 && stableLidDis <= disc3)
      discNum = 2;
    else
      discNum = 3;
    preLidDis = lidDis;
    if (discNum == 3)
    {
      intakeControlOverriden = true;
      intake(-100);
      if (!preState)
        timerOffset = TIMER;
      if (TIMER - timerOffset >= 250)
        preOpenLid = true;
    }
    else
    {
      intakeControlOverriden = false;
    }
    preState = intakeControlOverriden;
    delay(10);
  }
  return 1;
}

//  100, 640
//  120, 656
//  140, 670
//  160, 690
//  181, 740
//  200, 770
//  220, 830
//  240, 848
//  260, 887
//  280, 953
//  300, 1120

float encoderOutset = 0;
float getEncoder(float x)
{
  float ret;
  ret = 0.00656134 * x * x - 0.638719 * x + 635.464 + 40;
  ret *= discNum / 3;
  ret = fmin(ret, 1200);
}

int disCalculator()
{
  float lengthFactor;
  while (1)
  {
    lengthFactor = (pow(getEncoder(getDistance(shootTarget)), 2) - pow(getTriggerEncoder(), 2)) * 0.01;
    if (autoTrigger)
      // triggerDisTarget = fmin(getEncoder(getDistance(shootTarget)) + lengthFactor, 1200);
      triggerDisTarget = fmin(getEncoder(getDistance(shootTarget)), 1400) + encoderOutset;
    delay(10);
  }
  return 1;
}

int disControl()
{
  while (1)
  {
    if (adjustTrigger)
    {
      auto pid = PID();
      pid.setCoefficient(5.5, 0, 0.17);
      pid.setIstart(50);
      pid.setErrorTolerance(3);
      pid.setdTolerance(0.02);
      while (adjustTrigger)
      {
        pid.setTarget(triggerDisTarget);
        pid.update(getTriggerEncoder());
        // triggerReady = abbs(getTriggerEncoder() - triggerDisTarget) <= 5;
        triggerReady = pid.targetArrived();
        // cout<<triggerReady<<" "<<getTriggerEncoder()<<" "<<pid.getVelocity()<<endl;
        moveTrigger(pid.getOutput() + getTriggerEncoder() * 0.01 + 5);
        delay(10);
      }
    }
    delay(10);
  }
  return 1;
}

void hit()
{
  bool inAutoAdjust = autoTrigger;
  adjustTrigger = 0;
  if (inAutoAdjust)
    autoTrigger = 0;
  moveTrigger(-5);
  delay(50);
  releaseTrigger();
  delay(250);
  // if (triggerDisTarget > 800)
  //   delay(200);
  lockTrigger();
  delay(100);
  resetTriggerEncoder();
  encoderOutset = 0;
  adjustTrigger = 1;
  if (inAutoAdjust)
    autoTrigger = 1;
}

void setShootTarget(V2 in, float outset = 0)
{
  shootTarget = in;
  encoderOutset = outset;
}

void shoot(Eigen::Vector2f target = shootTarget)
{
  // shootTarget = target;
  lookAt(target, 0, 2, 1);
  intake(-100);
  float timerOffset = TIMER;
  while (!triggerReady)
    delay(10);
  if (!preOpenLid)
    while ((TIMER - timerOffset) < 250)
      delay(10);
  hit();
}

void shootWithoutAiming()
{
  intake(-100);
  float timerOffset = TIMER;
  // while (!triggerReady)
  //   delay(10);
  if (!preOpenLid)
    while ((TIMER - timerOffset) < 250)
      delay(10);
  hit();
}

int ensureExpansion()
{
  float timerOffset = TIMER;
  while ((TIMER - timerOffset) <= 59900)
    delay(10);
  expand(1);
  return 1;
}

#endif