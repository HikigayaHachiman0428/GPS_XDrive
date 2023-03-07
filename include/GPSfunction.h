#ifndef _GPS_H_
#define _GPS_H_
#include "definitions_and_declarations.h"
#include "headers.h"
#undef __ARM_NEON__
#undef __ARM_NEON
#include <eigen-3.4.0/Eigen/Dense>

using namespace vex;
float getRadian(float deg) { return deg / 180 * M_PI; }
float getStandardRadian(float deg)
{ // within unit circle
  float ret = getRadian(deg);
  while (ret >= 2 * M_PI)
    ret -= 2 * M_PI;
  while (ret <= -2 * M_PI)
    ret += 2 * M_PI;
  return ret;
}

Eigen::Vector2f rotateVector(Eigen::Vector2f original, float rotDeg)
{
  rotDeg = getStandardRadian(rotDeg);
  Eigen::Vector2f ret;
  ret << original[0] * cos(rotDeg) - original[1] * sin(rotDeg),
      original[0] * sin(rotDeg) + original[1] * cos(rotDeg);
  return ret;
}

float getRadianOnUnitCircle(Eigen::Vector2f vector)
{
  if (vector[0] > 0)
    return atan(vector[1] / vector[0]);
  else if (vector[0] < 0)
    return M_PI + atan(vector[1] / vector[0]);
  else if (vector[1] > 0)
    return M_PI / 2;
  else
    return 3 * M_PI / 2;
}

float processGyroDifference(float target)
{
  float difference =
      (getStandardRadian(target) - getStandardRadian(GYRO)) / M_PI *
      180;
  if (fabs(difference) < 1e-4)
    return 0;
  return difference;
}

// 得到两个vector的夹角
float getTheta(Eigen::Vector2f from, Eigen::Vector2f to)
{
  return sign(getRadianOnUnitCircle(to) - getRadianOnUnitCircle(from)) *
         ((acos(from.dot(to) / (from.norm() * to.norm()))) / M_PI) * 180;
}

Eigen::Vector2f localSpeed, globalSpeed, CoR, Omnipos;
float encoderRatio = 60 / 17.15; // deg -> cm

Eigen::Vector2f friendlyHighGoal, scoringHighGoal;

void setCoordinates()
{
  Omnipos << -150, 0;
  CoR = Omnipos + Omnipos_To_CoR;
  friendlyHighGoal << -132, 132; // the coordinates of the high goal
  scoringHighGoal << 132, -132;
}

// int GPScoordinator() {
//   float preX, curX, preY, curY, preGyro;
//   startGyro = GYRO;
//   preGyro = GYRO;
//   curX = -encoderx.rotation(deg);
//   curY = -encodery.rotation(deg);
//   preX = curX;
//   preY = curY;
//   Omnipos_To_CoR << distancex, distancey;
//   setCoordinates();
//   while (1) {
//     curX = -encoderx.rotation(deg);
//     curY = -encodery.rotation(deg);
//     // if (!(curX - preX) && !(curY - preY))
//     //   continue;
//     localSpeed << -(curX - preX) * encoderRatio * sin(M_PI/4) -
//                       (curY - preY) * encoderRatio *
//                           cos(M_PI/4),
//         (curY - preY) * encoderRatio * sin(M_PI/4) +
//             (curX - preX) * encoderRatio * sin(M_PI/4);

//     // globalSpeed = rotateVector(localSpeed, (GYRO + preGyro) / 2
//     -
//     //                                            startGyro);
//     globalSpeed = rotateVector(localSpeed, GYRO -
//                                                 startGyro);
//     Omnipos += globalSpeed;
//     // CoR = Omnipos + rotateVector(Omnipos_To_CoR,
//     //                              (GYRO + preGyro) / 2 -
//     startGyro); CoR = Omnipos + rotateVector(Omnipos_To_CoR,
//                                   GYRO - startGyro);
//     preX = curX;
//     preY = curY;
//     preGyro = GYRO;
//     Brain.Screen.printAt(10, 20, "Encoder:%.2f %.2f", encoderx.rotation(deg),
//                          encodery.rotation(deg));
//     Brain.Screen.printAt(10, 40, "Omnipos:%.2f %.2f", Omnipos[0],
//     Omnipos[1]); Brain.Screen.printAt(10, 60, "CoRX:%.2f %.2f", CoR[0],
//     CoR[1]); wait(dt, msec);
//   }
//   return 1;
// }

int GPScoordinator()
{
  float preX, curX, preY, curY, averageGYRO;
  // startGyro = Gyro.rotation(deg);
  float preGYRO = startGyro;
  curX = -encoderx.rotation(deg);
  curY = -encodery.rotation(deg);
  preX = curX;
  preY = curY;
  setCoordinates();
  while (1)
  {
    averageGYRO = GYRO;
    // averageGYRO = (GYRO + preGYRO) / 2;
    curX = -encoderx.rotation(deg);
    curY = -encodery.rotation(deg);
    localSpeed << getRadian(curY - preY) * encoderRatio * sin(getRadian(45)) +
                      getRadian(curX - preX) * encoderRatio * sin(getRadian(45)),
        getRadian(curX - preX) * encoderRatio * sin(getRadian(45)) -
            getRadian(curY - preY) * encoderRatio *
                cos(getRadian(45));
    // globalSpeed = rotateVector(localSpeed, (Gyro.rotation(deg) - startGyro));
    globalSpeed = rotateVector(localSpeed, (averageGYRO - startGyro));
    Omnipos += globalSpeed;
    // CoR = Omnipos + rotateVector(Distance, (Gyro.rotation(deg) - startGyro));
    CoR = Omnipos + rotateVector(Omnipos_To_CoR, (averageGYRO - startGyro));
    preX = curX;
    preY = curY;
    preGYRO = GYRO;
    // Brain.Screen.printAt(10, 20, "Encoder:%.2f %.2f", encoderx.rotation(deg),
    //                     encodery.rotation(deg));

    Brain.Screen.printAt(10, 20, "LocalSpeed:%.2f %.2f", localSpeed[0],
                         localSpeed[1]);
    Brain.Screen.printAt(10, 40, "Omnipos:%.2f %.2f", Omnipos[0], Omnipos[1]);

    Brain.Screen.printAt(10, 60, "CoRX:%.2f %.2f", CoR[0], CoR[1]);
    Brain.Screen.printAt(10, 80, "Velocity:%.2f %.2f", globalSpeed[0],
                         globalSpeed[1]);

    delay(10);
  }
  return 1;
}

float lookAtCalc(Eigen::Vector2f target, float bodyAngle)
{
  target -= CoR;
  float theta;
  if (target[1] == 0)
    theta = M_PI / 2;
  else
    theta = atan(target[1] / target[0]);
  if (target[0] < 0)
    theta += M_PI;
  return (theta / M_PI * 180) + bodyAngle;
}

// target是目标的坐标向量，bodyAngle是想要面对target的部位和目前机器人的面向的偏转角度
void lookAt(Eigen::Vector2f target, float bodyAngle = 0, float tolerance = 2, float dtolerance = 1)
{
  OLDPIDGyroTurn(lookAtCalc(target, bodyAngle), tolerance, dtolerance);
}

Eigen::Vector2f getMove(Eigen::Vector2f target)
{
  Eigen::Vector2f move;
  move = target - Omnipos;
  return move;
}

void moveChasis(Eigen::Vector4f chasis)
{
  moveMotor(LF, chasis[0]);
  moveMotor(LB, chasis[1]);
  moveMotor(RF, chasis[2]);
  moveMotor(RB, chasis[3]);
}

Eigen::Vector4f fit(Eigen::Vector4f ori, float MAX)
{
  for (int i = 0; i < 4; ++i)
  {
    if (abbs(ori[i]) > MAX)
      ori *= abbs(MAX / ori[i]);
  }
  return ori;
}

Eigen::Vector4f getTravel(Eigen::Vector2f cur, Eigen::Vector2f target) // global speed 转成 local speed
{
  Eigen::Vector2f toTarget = target - cur;
  toTarget = rotateVector(toTarget, 90 - GYRO);
  // cout << toTarget.transpose() << endl;
  Eigen::Vector4f distance = Eigen::Vector4f(toTarget[0] + toTarget[1], -toTarget[0] + toTarget[1], toTarget[0] - toTarget[1], -toTarget[0] - toTarget[1]);
  // cout << "cur:" << cur.transpose() << endl;
  // cout << "Travel Distance: " << distance.transpose() << endl;
  return distance;
}

Eigen::Vector4f mixSpeed(Eigen::Vector4f travelSpeed, float rotateSpeed)
{
  float maxv = rotateSpeed > 0 ? (100 - rotateSpeed) : (-100 - rotateSpeed);
  for (int i = 0; i < 4; ++i)
  {
    if (fabs(travelSpeed[0] + rotateSpeed) > 100)
    {
      travelSpeed = (maxv / travelSpeed[i]) * travelSpeed;
    }
  }
  return travelSpeed + Eigen::Vector4f(rotateSpeed, rotateSpeed, rotateSpeed, rotateSpeed);
}

class aimer
{
private:
  float rotateV;
  float driveV;
  bool prediction;
  int count;
  Eigen::Vector2f lockTarget;
  Eigen::Vector2f predictedTarget;
  void setPredictedTarget()
  {
    if (!prediction)
      predictedTarget = lockTarget;
    else
    {
      float kPredict = (16.3 + (float)(CoR - lockTarget).norm() * 0.1);
      predictedTarget = lockTarget - globalSpeed * kPredict;
    }
  }

public:
  PID aimPID;
  bool Locked;
  aimer() : rotateV(0), Locked(false), count(0), prediction(false)
  {
    lockTarget = scoringHighGoal;
    predictedTarget = scoringHighGoal;
    aimPID = PID();
    aimPID.setCoefficient(3, 5, 0.4);
    aimPID.setErrorTolerance(2);
    aimPID.setIMin(0);
    aimPID.setLim(70);
  }
  void setLockTarget(Eigen::Vector2f t) { lockTarget = t; }
  void setDriveV(Eigen::Vector4f c)
  {
    for (int i = 0; i < 4; ++i)
    {
      driveV = max(driveV, abbs(c[i]));
    }
  }
  void setPrediction(bool in) { prediction = in; }
  float getAimRotate() { return rotateV; }
  float getGyroError() { return processTarget(lookAtCalc(predictedTarget, 0)) - GYRO; }
  void update()
  {
    setPredictedTarget();
    // cout<<predictedTarget.transpose()<<endl;
    // autoTrigger = true;
    // triggerDisTarget = getEncoder(getDistance(predictedTarget));
    if (autoTrigger)
      shootTarget = predictedTarget;
    float tar = lookAtCalc(predictedTarget, 0);
    aimPID.setTarget(processTarget(tar));
    // aimPID.setLim(fmin(driveV*0.7,70));
    // if (abbs(processTarget(tar) - GYRO) >= 30)
    //   aimPID.setLim(90);
    if (abbs(processTarget(tar) - GYRO <= 10))
      aimPID.setLim(70);
    aimPID.update(GYRO, Gyro.gyroRate(zaxis, dps) / 100);
    // if (driveV < 20 && aimPID.getOutput() < 10)
    //   rotateV = 0;
    rotateV = aimPID.getOutput();
    if (aimPID.targetArrived())
    {
      count++;
    }
    else
    {
      count = 0;
    }
    if (count >= 5)
    {
      Locked = true;
      // rotateV = 0;
    }
    else
    {
      Locked = false;
    }
  }
};

#define moveType_LOCK 1
void Move(float finalx, float finaly, float finalGyro, bool stop = 1,
          float vratio = 1, Eigen::Vector2f move = Eigen::Vector2f(0, 0), // move from CoR at starting position
          int moveType = 0, float ctolerance = 5, float dtolerance = 2)
{
  auto moveAimer = aimer();
  moveAimer.aimPID.setCoefficient(3, 0, 10.4);
  Eigen::Vector4f chasis = V4(0, 0, 0, 0);
  Eigen::Vector2f cur = CoR + rotateVector(move, GYRO - startGyro);
  Eigen::Vector2f target = Eigen::Vector2f(finalx, finaly);
  finalGyro = processTarget(finalGyro);
  float kpTravel = 10.8, kdTravel = 2.6, kiTravel = 0.12, iTravel = 0;
  if (!stop)
    kpTravel = 20, kdTravel = -10, kiTravel = 0.42, iTravel = 0;
  if (moveType == moveType_LOCK)
    ;
  kdTravel = 14.6;
  float iTravelRange = 10;
  Eigen::Vector4f travelDirection = getTravel(cur, target);
  travelDirection.normalize();
  float travelError = (cur - target).norm(), preTravelError = 0;
  float gyroError = finalGyro - GYRO, preGyroError = 0;
  float kpGyro = 9.5, kiGyro = 0.1, kdGyro = 12.5, iGyro = 0;
  float iGyroRange = 10;
  float GyroRatio = fabs((gyroError / 100) / (fabs(gyroError) / 100 + travelError / 50)); // travel : total
  bool arrived = false;
  if (moveType == moveType_LOCK)
  {
    moveAimer.setLockTarget(V2(finalx, finaly));
    autoTrigger = false;
  }
  while (!arrived)
  {
    cur = CoR + rotateVector(move, GYRO - startGyro);
    // cout<<"cur:" <<cur.transpose()<<endl;
    travelError = (cur - target).norm();
    if (abbs(travelError) <= ctolerance)
    {
      if (!stop)
        arrived = true;
      else if ((moveType != moveType_LOCK)
               // && abbs(gyroError) <= dtolerance
      )
        arrived = true;
      else if (moveType == moveType_LOCK)
        arrived = true;
    }
    travelDirection = getTravel(cur, target);
    travelDirection.normalize();
    if (abbs(travelError) < iTravelRange)
      iTravel += travelError * 0.01;
    if (preTravelError * travelError <= 0)
      iTravel = 0;
    Eigen::Vector4f movev = travelDirection * (travelError * kpTravel + (travelError - preTravelError) * kdTravel + iTravel * kiTravel);
    movev = fit(movev, 100);
    // cout<<"movev:"<<movev.transpose()<<endl;
    Eigen::Vector4f rotatev;
    // cout << "TError: " << travelError << " movev:" << (travelError * kpTravel - (travelError - preTravelError) * 0.01 * kdTravel + iTravel * kiTravel) << endl;
    gyroError = finalGyro - GYRO;
    GyroRatio = fabs((gyroError / 90) / (fabs(gyroError) / 90 + travelError / 50)); // travel : total
    // cout << GyroRatio * 100 << endl;

    if (abbs(gyroError) < iGyroRange)
      iGyro += gyroError * 0.01;
    if (preGyroError * gyroError <= 0)
      iGyro = 0;
    float rotateV = -1 * (gyroError * kpGyro + (gyroError - preGyroError) * kdGyro * 0.01 + iGyro * kiGyro);
    rotatev = Eigen::Vector4f(rotateV, rotateV, rotateV, rotateV);
    rotatev = fit(rotatev, 100 * GyroRatio);
    // cout << "GError: " << gyroError << " rotatev:" << rotatev[0] << endl;
    switch (moveType)
    {
    case 0:
      chasis = mixSpeed(movev, rotatev[0]);
      break;
    case moveType_LOCK:
    {
      moveAimer.update();
      chasis = mixSpeed(movev, -moveAimer.getAimRotate());
      break;
    }
    }
    chasis *= vratio;
    // cout << chasis[0] << " " << chasis[1] << " " << chasis[2] << " " << chasis[3] << endl;
    moveChasis(chasis);
    preTravelError = travelError;
    preGyroError = gyroError;
    // cout<<abbs(kdGyro * Gyro.gyroRate(zaxis, dps) / 100)<<endl;
    delay(10);
  }
  autoTrigger = true;
  driveForward(0);
}

enum moveMode
{
  DRIFT = 1,
  HASTY,
  PRECISE,
  LOCK
};

void xDriveMove(float finalx, float finaly, float finalGyro, float vratio = 1, moveMode mode = DRIFT, V2 move = V2(0, 0))
{
  float axisTolerance, GyroTolerance;
  finalGyro = processTarget(finalGyro);
  PID movePID = PID();
  PID rotatePID = PID();
  auto moveAimer = aimer();
  float unitDriveTime, unitRotateTime, drivev;
  V2 tar = V2(finalx, finaly);
  V2 cur = CoR + rotateVector(move, GYRO);
  // V2 startPos = cur;
  movePID.setTarget(tar);
  rotatePID.setTarget(processTarget(finalGyro));
  float gyroRatio = ((abbs(processTarget(finalGyro) - GYRO)) * unitRotateTime) / (abbs(processTarget(finalGyro) - GYRO) * unitRotateTime + abbs((tar - cur).norm()) * unitDriveTime);
  V4 moveV, driveV, rotateV;
  moveV = driveV = rotateV = V4(0, 0, 0, 0);
  bool arrived = false;
  switch (mode)
  {
  case (DRIFT):
  {
    movePID.setTarget(V2(finalx, finaly));
    movePID.setCoefficient(10.8, 0.12, 2.6);
    movePID.setErrorTolerance(axisTolerance);
    movePID.setIstart(10);
    movePID.setdTolerance(2);
    rotatePID.setTarget(finalGyro);
    rotatePID.setCoefficient(9.5, 0.1, 12.5);
    rotatePID.setErrorTolerance(GyroTolerance);
    rotatePID.setdTolerance(1e9);
    rotatePID.setIstart(10);
    unitDriveTime = 50;
    unitRotateTime = 90;
    break;
  }
  case (HASTY):
  {
    movePID.setTarget(V2(finalx, finaly));
    movePID.setCoefficient(20, 0.42, -10);
    movePID.setErrorTolerance(axisTolerance);
    movePID.setIstart(20);
    movePID.setdTolerance(1e9);
    rotatePID.setTarget(finalGyro);
    rotatePID.setCoefficient(9.5, 0.1, 12.5);
    rotatePID.setErrorTolerance(GyroTolerance);
    rotatePID.setdTolerance(1e9);
    rotatePID.setIstart(10);
    unitDriveTime = 50;
    unitRotateTime = 90;
    break;
  }
  case (PRECISE):
  {
    movePID.setTarget(V2(finalx, finaly));
    movePID.setCoefficient(10.8, 0.12, 2.6);
    movePID.setErrorTolerance(axisTolerance);
    movePID.setIstart(10);
    movePID.setdTolerance(2);
    rotatePID.setTarget(finalGyro);
    rotatePID.setCoefficient(9.5, 0.1, 12.5);
    rotatePID.setErrorTolerance(GyroTolerance);
    rotatePID.setdTolerance(1e9);
    rotatePID.setIstart(10);
    unitDriveTime = 50;
    unitRotateTime = 90;
    break;
  }
  case (LOCK):
  {
    movePID.setTarget(V2(finalx, finaly));
    moveAimer.aimPID.setCoefficient(3, 0, 10.4);
    moveAimer.setLockTarget(V2(finalx, finaly));
    movePID.setCoefficient(10.8, 0.12, 2.6);
    movePID.setErrorTolerance(axisTolerance);
    movePID.setIstart(10);
    movePID.setdTolerance(2);
    unitDriveTime = 50;
    unitRotateTime = 90;
    break;
  }
  }
  while (!arrived)
  {
    switch (mode)
    {
    case (DRIFT, HASTY, LOCK):
    {
      arrived = movePID.targetArrived();
    }
    case (PRECISE):
    {
      arrived = movePID.targetArrived() && rotatePID.targetArrived();
    }
    }
    cur = CoR + rotateVector(move, GYRO);
    // get global travel v in pid
    movePID.update(cur);
    drivev = movePID.getOutput();
    // convert to local v
    driveV = getTravel(cur, tar);
    driveV.normalize();
    driveV = driveV * drivev;
    driveV = fit(driveV, 100);
    // get rotate v in pid
    if (mode != LOCK)
    {
      rotatePID.setLim(100 * gyroRatio);
      rotatePID.update(GYRO, Gyro.gyroRate(zaxis, dps) * 0.01);
      rotateV = V4(-rotatePID.getOutput(), -rotatePID.getOutput(), -rotatePID.getOutput(), -rotatePID.getOutput());
    }
    // get rotate v in aimer
    else
    {
      moveAimer.update();
      rotateV = V4(-moveAimer.getAimRotate(), -moveAimer.getAimRotate(), -moveAimer.getAimRotate(), -moveAimer.getAimRotate());
    }
    moveV = mixSpeed(driveV, rotateV[0]);
    moveV = moveV * vratio;
    moveChasis(moveV);
    cout << moveV.transpose() << endl;
    // cout<<movePID.getVelocity()<<endl;
    delay(10);
  }
  driveForward(0);
}
#endif