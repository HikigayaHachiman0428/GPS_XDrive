/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Talia                                                     */
/*    Created:      Tue Jan 17 2023                                           */
/*    Description:  GPS v1.0                                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "autonomous.h"
#include "tasks.h"
using namespace vex;
competition Competition;
aimer AIMER;
void vexcodeInit()
{
  // Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(3, 1);
  // cPrint("/*---------------*/");
  // cPrint("/*  CALIBRATING  */");
  // cPrint("/*  DO NOT MOVE  */");
  // cPrint("/*---------------*/");
  Gyro.calibrate(5000);
  // delay(5000);
  setStartGyro(0);

  encoderx.resetRotation();
  encodery.resetRotation();
  LF.resetPosition();
  LB.resetPosition();
  RF.resetPosition();
  RB.resetPosition();
  // releaseTrigger();
  // delay(500);
  lockTrigger();
  // delay(500);
  triggerLeft.resetPosition();
  triggerRight.resetPosition();
  liftIntake(false);
  expand(false);
  // Controller1.Screen.clearScreen();
}

void Auto()
{
  // float startTime = TIMER;
  switch (autoRoutine)
  {
  case 0:
    // test();
    break;
  case 1:
    AWP();
    break;
  case 2:
    leftPath();
    break;
  case 3:
    rightPath();
    break;
  case 4:
    skills();
    break;
  }
  // cout << "total time : " << (TIMER - startTime) / 1000 << " s" << endl;
}

void Ch()
{
  switch (ch_state)
  {
  case 0: // 四轮
    LB.spin(fwd, 120 * (Ch1 + Ch3), voltageUnits::mV);
    LF.spin(fwd, 120 * (Ch1 + Ch3), voltageUnits::mV);
    RB.spin(fwd, 120 * (Ch1 - Ch3), voltageUnits::mV);
    RF.spin(fwd, 120 * (Ch1 - Ch3), voltageUnits::mV);
    break;
  case 1: // Xdrive 机器第一人称
    autoTrigger = false;
    moveMotor(LF, Ch1 + Ch3 + Ch4);
    moveMotor(LB, Ch1 + Ch3 - Ch4);
    moveMotor(RF, Ch1 - Ch3 + Ch4);
    moveMotor(RB, Ch1 - Ch3 - Ch4);
    break;
  case 2:
  { // Xdrive 机器第三人称
    autoTrigger = false;
    float g = getRadian(90) - getRadian(GYRO);
    float vx = cos(g) * (Ch4)-sin(g) * Ch3;
    float vy = cos(g) * Ch3 + sin(g) * (Ch4);
    moveMotor(LF, Ch1 + vy + vx);
    moveMotor(LB, Ch1 + vy - vx);
    moveMotor(RF, Ch1 - vy + vx);
    moveMotor(RB, Ch1 - vy - vx);
    break;
  }
  case 3: // 第一人称自锁
  {
    autoTrigger = true;
    Eigen::Vector4f c;
    c[0] = Ch3 + Ch4;
    c[1] = Ch3 - Ch4;
    c[2] = Ch4 - Ch3;
    c[3] = -Ch4 - Ch3;
    c = c * -1;
    AIMER.setDriveV(c);
    // AIMER.aimPID.setLim(50);
    AIMER.update();

    float rotateSpeed = -AIMER.getAimRotate();
    // cout<<-rotateSpeed<<endl;
    c = mixSpeed(c, rotateSpeed);
    // cout<<c.transpose()<<endl;
    moveMotor(LF, c[0]);
    moveMotor(LB, c[1]);
    moveMotor(RF, c[2]);
    moveMotor(RB, c[3]);
    break;
  }
  case 4:
  { // 第三人称自锁
    autoTrigger = true;
    Eigen::Vector4f c;
    float g = getRadian(90) - getRadian(GYRO);
    float vx = cos(g) * (Ch4)-sin(g) * Ch3;
    float vy = cos(g) * Ch3 + sin(g) * (Ch4);
    c[0] = vx + vy;
    c[1] = vy - vx;
    c[2] = vx - vy;
    c[3] = -vx - vy;
    AIMER.setDriveV(c);
    AIMER.update();
    float rotateSpeed = -AIMER.getAimRotate();
    // cout<<-rotateSpeed<<endl;
    c = mixSpeed(c, rotateSpeed);
    // cout<<c.transpose()<<endl;
    moveMotor(LF, c[0]);
    moveMotor(LB, c[1]);
    moveMotor(RF, c[2]);
    moveMotor(RB, c[3]);
    break;
  }
  }
}

void usercontrol()
{
  // task OM(skills);
  // while (!BA)
  // {
  //   delay(10);
  // }
  // OM.stop();
  AIMER = aimer();
  bool preL1 = false, preL2 = false, preR1 = false, preR2 = false,
       preLeft = false, preRight = false, preUp = false, preDown = false, preBX = false, preBY = false, preBA = false;
  bool lockMode = false, ThirdMode = false, adjustTrigger = true, autoTrigger = false, trigger = false, firstRumble = true, preOpenLid = false;
  float userStartTime = TIMER;
  resetTriggerEncoder();
  triggerDisTarget = 500;
  liftIntake(false);
  while (1)
  {
    if (inOneMinute)
    {
      if (CoR[0] < CoR[1])
      {
        shootTarget = friendlyHighGoal;
        AIMER.setLockTarget(friendlyHighGoal);
      }
      else
      {
        shootTarget = scoringHighGoal;
        AIMER.setLockTarget(scoringHighGoal);
      }
    }
    else
    {
      AIMER.setLockTarget(scoringHighGoal);
      shootTarget = scoringHighGoal;
    }
    // cout << "holdTrigger " << holdTrigger << endl;
    if (autoRoutine == 4)
      inOneMinute = true;
    else
      inOneMinute = false;
    // if (abbs(GYRO - lookAtCalc(shootTarget, 0)) <= 3)
    //   Controller1.rumble("**.**");
    // cout << abbs(GYRO - lookAtCalc(shootTarget, 0)) << endl;
    if (BX && !preBX)
    {
      lookAt(shootTarget);
      lockMode = !lockMode;
      autoTrigger = !autoTrigger;
    }
    if (UP && !preUp)
      ThirdMode = !ThirdMode;
    if (lockMode && ThirdMode)
    {
      ch_state = 4;
    }
    if (!lockMode && ThirdMode)
      ch_state = 2;
    if (!lockMode && !ThirdMode)
      ch_state = 1;
    if (lockMode && !ThirdMode)
    {
      ch_state = 3;
    }
    Ch();
    if (!preLeft && LEFT)
    {
      if (autoRoutine == 0)
        autoRoutine = 4;
      else
        autoRoutine--;
    }
    if (!preRight && RIGHT)
    {
      if (autoRoutine == 4)
        autoRoutine = 0;
      else
        autoRoutine++;
    }
    if ((TIMER - userStartTime) >= 95000)
    {
      if (firstRumble)
      {
        Controller1.rumble(".*** . ..*. ..*. . .*. *.** .. ... ** *.** ... . *..* ... .*.. .* ...* .");
        firstRumble = false;
      }
      if (RIGHT && BY && (!preRight || !preBY)
          // && (TIMER - userStartTime) >= 95000
      )
      {
        expansion.set(1);
      }
    }

    if (BA && !preBA)
    {
      // rightPath();
      // skills();
      // Move(50, 0, 30, 0);
      // shoot(friendlyHighGoal);
      // lookAt(V2(-170, -95), 180, 10, 2);
      // Move(-77, -25, -30, 0,1, CoR_To_Intake, 1);
      // intake(100);
      // Move(-125, -113, -30);
      // delay(100);
      liftIntake(!intakeUplift.value());
      // Auto();
      //  lookAt(V2(-120, 120));
      //  float t = TIMER;
      //  PIDGyroTurn(90 + GYRO);
      //  cout << GYRO << " " << TIMER - t << endl;

      // encoderIndex(100, 100);
    }

    if (L1() && !preL1)
    {
      shootWithoutAiming();
      moveMotor(ind, 10);
      delay(50);
      moveMotor(ind, 0);
      lockMode = false;
    }
    if (!trigger && L2())
    {
      autoTrigger = false;
      if (R1())
        triggerDisTarget = 660;
      // triggerDisTarget = 500;
      if (R2())
        triggerDisTarget = 830;
      // triggerDisTarget = 800;
      triggerDisTarget -= 10 * ((UP && !preUp) - (DOWN && !preDown));
    }
    if (!trigger && !L2())
    {
      intake(100 * (R1() - R2()));
    }
    preL1 = L1(), preR1 = R1(), preL2 = L2(), preR2 = R2(), preLeft = LEFT,
    preRight = RIGHT, preUp = UP, preBX = BX, preBY = BY, preDown = DOWN, preBA = BA;
    delay(10);
  }
}

int main()
{
  vexcodeInit();
  // usercontrol();
  task GYROREADER(GyroReader);
  task GPSCOORDINATOR(GPScoordinator);
  // task LOCKAIM(Aimer);
  task TRIGGER(triggerControl);
  task DISCONTROL(disControl);
  task AUTOCAL(disCalculator);
  // DISCONTROL.suspend();
  delay(200);
  Competition.autonomous(Auto);
  Competition.drivercontrol(usercontrol);
  delay(200);
  while (1)
  {
    /* Brain Print */
    // cout << autoTrigger << " "<<ch_state<<endl;
    Brain.Screen.printAt(10, 120, "Trigger:%.2f", triggerLeft.position(deg));
    Brain.Screen.printAt(10, 140, "Trigger Target:%.2f", triggerDisTarget);
    Brain.Screen.printAt(10, 160, "Gyro:%.2f", GYRO);
    // Brain.Screen.printAt(10, 180, "AutoRountine:%d", autoRoutine);
    BrainPrintRoutine();
    /* Controller Print */
    Controller1.Screen.setCursor(3, 1);
    // Controller1.Screen.print("    %d:%d    ", (int)TIMER / 60000,
    //                          ((int)TIMER) % 60000 / 1000);
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("Battery:%d", (int)Brain.Battery.capacity());
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("Gyro:%.2f", readGyro());
    // Controller1.Screen.newLine();
    // Controller1.Screen.print("AutoRountine: %d", autoRoutine);
    // Controller1.Screen.print("CoR: %.1f %.1f", CoR[0], CoR[1]);
    // Controller1.Screen.newLine();
    controllerPrintRoutine();
    cout << (scoringHighGoal - CoR).norm() << " " << getTriggerEncoder() << endl;
    //  cout<<encoderx.position(deg)<<" "<<encodery.position(deg)<<endl;
    // cout<<CoR.transpose()<<" "<<GYRO<<endl;
    // cout << LF.power() << " " << LB.power() << " " << RF.power() << " " << RB.power() << endl;
    //  cout<<chasis[0]<<" "<<chasis[1]<<" "<<chasis[2]<<" "<<chasis[3]<<" "<<endl;
    delay(10);
  }
}
