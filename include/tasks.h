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
  return 0.00656134 * x * x - 0.638719 * x + 635.464 + 40;
  // return 900;
  // return -1.1750346185979898e-14*x*x*x*x*x*x*x*x*x+2.0073670492520317e-11*x*x*x*x*x*x*x*x-1.5048549381392062e-8*x*x*x*x*x*x*x+0.0000064941540199576705*x*x*x*x*x*x-0.001776948977230754*x*x*x*x*x+0.3195324230343142*x*x*x*x-37.74156618212961*x*x*x+2822.211247582093*x*x-121182.51731063976*x+2276237.465690938;;
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

int triggerControl()
{
  while (1)
  {
    if (trigger)
    {
      bool inAutoAdjust = autoTrigger;
      // while (!triggerReady)
      //   delay(10);
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
      trigger = 0;
      // moveMotor(ind, 10);
      // delay(50);
      // moveMotor(ind, 0);
    }
    delay(10);
  }
  return 1;
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
    while ((TIMER - timerOffset) < 200)
      delay(10);
  trigger = 1;
  while (trigger)
    delay(10);
}

void shootWithoutAiming()
{
  intake(-100);
  float timerOffset = TIMER;
  if (!preOpenLid)
    while ((TIMER - timerOffset) < 300)
      delay(10);
  while (!triggerReady)
    delay(10);
  trigger = 1;
  while (trigger)
    delay(10);
}

float changeAim() { locked = !locked; }
float aimSpeed;
float getAimSpeed() { return aimSpeed; }
int Aimer()
{
  // 282cm 2600
  // 150cm 2200
  while (1)
  {
    aimSpeed = 0;
    if (lockMode)
    {
      // cout<<CoR.transpose()<<endl;
      autoTrigger = true;
      // globalspeed = CoR - lastCoR;
      float lastglobaldis = 0, lockIntegral = 0, lastlockerror = 0, lockedcount = 0, error = 0;
      while (lockMode)
      {

        float predis = sqrt((scoringHighGoal[0] - CoR[0]) * (scoringHighGoal[0] - CoR[0]) + (scoringHighGoal[1] - CoR[1]) * (scoringHighGoal[1] - CoR[1]));
        Eigen::Vector2f predicttarget = scoringHighGoal - globalSpeed * (16.3 + predis * 0.1);
        // predicttarget << scoringHighGoal[0]-globalspeed[0]*(8.3+predis/10), scoringHighGoal[1]-globalspeed[1]*(8+predis/8.2);
        float dis = sqrt((predicttarget[0] - CoR[0]) * (predicttarget[0] - CoR[0]) + (predicttarget[1] - CoR[1]) * (predicttarget[1] - CoR[1]));
        // cout << dis << endl;
        // triggerDisTarget = getEncoder(predis);
        float globaldisspeed = dis - lastglobaldis;
        float target = lookAtCalc(predicttarget, 0);
        // cout << target << endl;
        error = processTarget(target) - GYRO;
        // cout << error << endl;
        float rotatepower = 0;

        // cout << CoR.transpose() << "   "<< globalRot<< endl;
        // cout << predicttarget.transpose()<<"  "<<target<<"  "<<error <<"   "<<dis<<endl;

        // 300 900
        // 200 710 1.35
        // y= 1.357x + 440

        // if (globalspeed.norm()<0.5) if (lastlockerror*error <= 0) lockIntegral = 0;
        float v = Gyro.gyroRate(zaxis, dps) / 100;
        // float v = error - lastlockerror;

        float kp = 4;
        if (globalSpeed.norm() > 1)
        {
          kp = kp * sqrt(sqrt(globalSpeed.norm()));
        }

        float ki = 0.05;
        float kd = 30;
        if (globalSpeed.norm() > 0.5)
        {
          kd = 30;
          ki = 0.05;
        }

        // if (globalspeed.norm()<0.2&&fabs(error)<0.3) {kp=0;kd=0;ki=0;}
        if (fabs(error) < 10)
        {
          lockIntegral += error;
        }
        if (error * lastlockerror <= 0)
        {
          lockIntegral = 0;
        }

        lastlockerror = error;
        lastglobaldis = dis;

        if (fabs(error) <= 1)
          lockedcount++;
        else
          lockedcount = 0;
        if (lockedcount >= 3)
          locked = true;
        else
          locked = false;

        // cout << error << "  "<<v <<endl;
        aimSpeed = abs(kp * error + ki * lockIntegral + kd * v) > 70 ? -sign(kp * error + ki * lockIntegral * kd * v) * 70 : -(kp * error + ki * lockIntegral + kd * v);
        // cout << aimSpeed << endl;
        delay(10);
      }
    }
    delay(10);
  }
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