#ifndef _AUTO_PATH_
#define _AUTO_PATH_
#include "auto_functions.h"
#include "tasks.h"
#include "definitions_and_declarations.h"
using namespace Eigen;
void controllerPrintRoutine()
{
  switch (autoRoutine)
  {
  case 0:
    Controller1.Screen.print("EMPTY ROUTINE");
    break;
  case 1:
    Controller1.Screen.print("AWP FROM LEFT");
    break;
  case 2:
    Controller1.Screen.print("LEFT ROUTINE");
    break;
  case 3:
    Controller1.Screen.print("RIGHT ROUTINE");
    break;
  case 4:
    Controller1.Screen.print("SKILLS");
    break;
  }
}
void BrainPrintRoutine()
{
  switch (autoRoutine)
  {
  case 0:
    Brain.Screen.printAt(10, 180, "EMPTY ROUTINE");
    break;
  case 1:
    Brain.Screen.printAt(10, 180, "AWP FROM LEFT");
    break;
  case 2:
    Brain.Screen.printAt(10, 180, "LEFT ROUTINE");
    break;
  case 3:
    Brain.Screen.printAt(10, 180, "RIGHT ROUTINE");
    break;
  case 4:
    Brain.Screen.printAt(10, 180, "SKILLS");
    break;
  }
}

void test()
{
  CoR << 150, -150;
  setShootTarget(scoringHighGoal);
  autoTrigger = 1;
  shootWithoutAiming();
}

void AWP()
{
  Omnipos << 90, 121;
  // vexDelay(50);
  // while (1) {cout<<globalPos.transpose()<<endl;vexDelay(50);}
  // intake(100);
  setShootTarget(scoringHighGoal);
  timerForward(100, 350);
  driveForward(20);
  encoderIndex(100, 100);
  timerForward(-100, 150);
  intake(100);
  lookAt(V2(125, 115));
  timerForward(80, 260);
  timerForward(-100, 300);
  Move(66, 120, -80, 0.7);
  // timerForward(-100,200);
  // Move(100,125,-80);
  // vexDelay(300);
  shoot(V2(132, -132));

  liftIntake(true);
  intake(100);
  Move(76, 110, -145);
  Move(40, 94, -145, 0.6);
  vexDelay(200);
  liftIntake(false);
  timerForward(-60, 100);
  timerForward(40, 400);
  vexDelay(1000);
  shoot(V2(132, -132));

  intake(100);
  Move(-90, -50, -140);
  // Move(-100,-20, -30);
  shoot(V2(132, -132));

  Move(-142, -110, 180);
  timerForward(50, 200);
  // timerForward(40, 100);
  driveForward(15);
  delay(33);
  encoderIndex(100, 100);
  timerForward(-100, 50);
  // Move(0,120,-75);
  // vexDelay(100);
  // discorrect = true;
  //  hitting = true;
  //  degarounded = false; punchPrep = true;
  //  PIDGyroTurnDegLookAt(132, -132, 0);
  //  while (hitting||punchPrep) {delay(10);}
}

void leftPath()
{
  int timerStart = TIMER;
  Omnipos << 98, 171;
  resetTriggerEncoder();
  triggerDisTarget = 500;
  // while (1) {cout<<globalPos.transpose()<<endl;vexDelay(50);}
  // intake(100);
  autoTrigger = false;
  setShootTarget(scoringHighGoal);
  // timerForward(-100, 100);
  driveForward(-25);
  delay(100);
  timerIndex(-100, 600);
  setShootTarget(scoringHighGoal);
  autoTrigger = true;
  intake(100);
  // cout<<1<<endl;
  driveForward(100);
  delay(50);
  driveForward(0);
  intake(100);
  // Move(115, 125, 0, 1, 1, CoR_To_Intake, 1);
  driveRotate(100);
  delay(75);
  lookAt(V2(123, 120), 0, 2, 1);
  timerForward(100, 300);
  delay(100);
  timerForward(-100, 200);
  Move(66, 130, -80, 0);
  // timerForward(-100,200);
  // Move(100,125,-80);
  // vexDelay(300);
  // driveRotate(100);
  // delay(50);
  driveRotate(100);
  delay(50);
  cout << 0 << endl;
  lookAt(V2(130, -125), 0, 2, 1);
  cout << getTriggerEncoder() << endl;
  adjustTrigger = 0;
  cout << adjustTrigger << endl;
  shootWithoutAiming();
  autoTrigger = 1;
  intake(100);
  liftIntake(true);
  driveRotate(100);
  delay(50);
  intake(100);
  lookAt(V2(90, 90), 0, 3, 1);
  Move(81, 98, 125, 1, 1, CoR_To_Intake, 1);
  delay(200);
  liftIntake(false);
  delay(800);
  driveRotate(100);
  delay(50);
  shoot(V2(120, -120));
  driveForward(-100);
  delay(100);

  intake(100);
  liftIntake(true);
  intake(100);
  Move(28, 92, 0, 1, 1, CoR_To_Intake, 1);
  delay(200);
  liftIntake(false);
  delay(800);
  driveRotate(-100);
  delay(50);
  shoot(V2(120, -120));
  delay(50);
  intake(100);
  // DC.stop();
  // punTarget = 850;
  Move(-30, 43, 150);
  Move(-32, 120, 145, 0.8);
  Move(0, 117, -50);
  // vexDelay(100);
  // discorrect = true;
  // delay(500);
  // shoot(V2(132, -132));
  lookAt(V2(120, -120), 0, 2, 1);
  moveMotor(ind, -100);
  float timerOffset = TIMER;
  while (!triggerReady)
    delay(10);
  while ((TIMER - timerOffset) < 200)
    delay(10);
  trigger = 1;
  while (trigger)
    delay(10);
  autoTrigger = false;
  cout << TIMER - timerStart << endl;
  while (1)
  {
    delay(10);
  }
}
void rightPath()
{
  preOpenLid = true;
  lidStart = 13;
  // set starting Ominpos
  float startTime = TIMER;
  intake(100);
  autoTrigger = true;
  while (getTriggerEncoder() < 500)
    delay(10);
  Move(-88, -30, 0, 0, 1, CoR_To_Intake, 1);
  // delay(300);
  while (!triggerReady)
    delay(10);
  shoot(V2(132, -135));
  setShootTarget(scoringHighGoal);
  /* {
     driveHorizontal(100);
     delay(550);
     intake(100);
     Move(-120, -115, -26, 1, 1, CoR_To_Intake, 1);
     autoTrigger = false;
     delay(100);
   }*/

  /* {
     // Move(-105, -82, -29, 0, 1, CoR_To_Intake);
     //  shoot(V2(132, -135));
     liftIntake(true);
     intake(100);
     Move(-93, -88, -42.5, 1, 1, CoR_To_Intake);
     delay(200);
     liftIntake(false);
     delay(1100);
     lookAt(V2(132, -132), 0, 2, 1);
     shootWithoutAiming();
     autoTrigger = 1;
   }*/

  /*{
    preOpenLid = false;
    Move(-150, -105.5, 0, 0);
    lookAt(V2(120, -115));
    preOpenLid = true;
    driveForward(-100);
    delay(100);
    intake(0);

    driveForward(-45);
    delay(80);
    // encoderIndex(100, 300);
    timerIndex(-100, 600);
    autoTrigger = true;
    // timerForward(-100, 100);
    driveForward(100);
    delay(100);
    intake(100);
  }*/

  /*{
    // driveForward(-100);
    // delay(220);
    // driveForward(0);
    intake(100);
    // while (getTriggerEncoder() < 500)
    //   delay(10);
    Move(-24, 33, 45, 1, 1, CoR_To_Intake, 1);
    // Move(-32, 32, 0, 0, 1);1
    // lookAt(V2(132, -130), 0);
    vexDelay(300);
    // delay(600);
    // return;
    // setShootTarget(scoringHighGoal, 10);
    shoot(V2(132, -132));
    // vexDelay(100);
  }*/
  {
    intake(100);
    preOpenLid = false;
    Move(-60, -60, -45, 0, 1, CoR_To_Intake);
    delay(200);
    timerForward(-100, 200);
    driveRotate(100);
    delay(100);
    Move(-33, -30, 0, -15, 1, CoR_To_Intake);
    delay(100);
    lookAt(V2(-60, 0));
    timerForward(100, 300);
  }

  /*{
    driveRotate(100);
    delay(100);
    intake(100);
    Move(-40, 32, 115, 0);
    Move(-120, 39, 155, 1, 0.8);
    driveRotate(-100);
    // Move(-130, 23, 15, 1)
    delay(200);

    // Move(-120, 0, -20, 1);
    // vexDelay(100);
    // shoot(V2(145  , -132));
    lookAt(V2(145, -132), 0, 2, 1);
    shootWithoutAiming();
    // autoTrigger = false;
  }
  */
  {
    preOpenLid = true;
    Move(-120, 10, -20);
    shoot(V2(125, -125));
    intake(100);
    driveRotate(100);
    delay(100);
    Move(-125, 44, 50, 0, 1, CoR_To_Intake);
    Move(-40, 42, 55, 0, 0.8, CoR_To_Intake);
    // delay(300);
    // lookAt(V2(-130, 130));
    // timerForward(50, 100);
    shoot(V2(130, -130));
  }
  {
    intake(100);
    Move(-27, 48, 140, 0);
    Move(-28, 130, 145, 0, 0.8);
    // delay(200);
    // driveForward(100);
    // timerForward(50, 100);
    Move(-15, 135, 180);
    // delay(100);
    // PIDGyroTurnDegLookAt(-132, 135, 0);
    // delay(200);
    shoot(V2(130, -135));
  }
  cout << "total Time:" << TIMER - startTime << endl;

  while (1)
  {
    delay(10);
  }
  // set starting CoR
}

int skills()
{
  task EE(ensureExpansion);
  float startTime = TIMER;
  autoTrigger = true;
  setShootTarget(friendlyHighGoal);
  {
    intake(100);
    // Move(-92, -26.6, -30, 0);
    Move(-90, -29.5, -34, 1, 1, CoR_To_Intake);
    delay(200);
    shoot(V2(-130, 130));
    delay(50);
  }
  cout << "first one + pre " << Brain.timer(msec) - startTime << endl;

  {
    setShootTarget(scoringHighGoal);
    liftIntake(true);
    intake(100);
    // Move(-100, -40, -75.6, 0);

    Move(-87.5, -85, -90, 1, 1, CoR_To_Intake, moveType_LOCK);
    delay(100);
    liftIntake(false);
    autoTrigger = true;
    // delay(100);
    // rageIntake();
    delay(1200);
    shoot(V2(125, -130));
    // delay(50);
    // while (1) {vexDelay(20);}
  }
  cout << "first stack " << Brain.timer(msec) - startTime << endl;

  setShootTarget(scoringHighGoal, 15);
  {
    intake(100);
    // driveRotate(-100);
    // delay(100);
    Move(-53, -53, 45, 0, 1, CoR_To_Intake);
    // Move(-135, -135, 225, 0, 1, CoR_To_Intake);
    Move(-100, -100, 225, 0, 1, CoR_To_Intake);
    Move(-150, -150, 225, 0, 1, CoR_To_Intake);
    // driveForward(100);
    // delay(200);
    // // timerForward(100, 200);
    // // timerForward(-100, 300);
    // driveForward(-100);
    // delay(500);
    // intake(0);
    driveForward(-100);
    delay(400);
    intake(0);
    Move(-160, -130, 170, 1, 1, CoR_To_Intake);
    // intake(0);
    timerForward(80, 350);
    driveForward(20);
    encoderIndex(100, 200);
    // intake(100);
    timerForward(-100, 200);
    // driveHorizontal(-100);
    // delay(300);
    Move(-105, -157, 270, 1, 1, CoR_To_Intake);
    timerForward(80, 200);
    driveForward(15);
    encoderIndex(100, 100);
    intake(100);
    // driveHorizontal(-85);
    // delay(200);
    timerForward(-100, 200);
    // driveForward(-100);
    // delay(300);
  }
  cout << "roller and corner " << Brain.timer(msec) - startTime << endl;

  {
    intake(100);
    // focusBlue;
    // Move(-65, -103, -10);
    // Move(-65,-59,100);
    shoot(V2(130, -140));
    // delay(50);
  }
  cout << "corner disks + 1 " << Brain.timer(msec) - startTime << endl;

  {
    setShootTarget(scoringHighGoal);
    liftIntake(true);
    intake(100);
    Move(-34.5, -96, 36.5, 1, 1, CoR_To_Intake, moveType_LOCK);
    delay(100);
    liftIntake(false);
    autoTrigger = true;
    // rageIntake();
    vexDelay(1200);
    shoot(V2(130, -130));
    // delay(100);
  }
  cout << "second stack " << Brain.timer(msec) - startTime << endl;

  {
    intake(100);
    setShootTarget(friendlyHighGoal);
    driveRotate(100);
    vexDelay(100);
    Move(-29, -40, 90, 0, 1, CoR_To_Intake);
    Move(-28, 45, 90, 0, 1, CoR_To_Intake);
    // Move(-30, 15, 135, 0, 1, CoR_To_Intake);
    // Move(-15,15,180);
    Move(-35, -3, 180, 0, 1, CoR_To_Intake);
    Move(-45, -3, 180, 0, 1, CoR_To_Intake);
    Move(-130, 10, 100);
    shoot(V2(-125, 125));
    // delay(50);
  }
  // while (hitting||punchPrep) {delay(10);}
  cout << "first drift " << Brain.timer(msec) - startTime << endl;

  {
    intake(100);
    Move(-127, 42, 50, 0, 1, CoR_To_Intake);
    Move(-33, 40, 55, 0, 0.8, CoR_To_Intake);
    // delay(300);
    // lookAt(V2(-130, 130));
    // timerForward(50, 100);
    shoot(V2(-130, 130));
    // delay(50);
  }
  cout << "first basket " << Brain.timer(msec) - startTime << endl;
  setShootTarget(friendlyHighGoal, -5);
  {
    intake(100);
    Move(-22, 48, 140, 0);
    Move(-23, 138, 145, 0, 0.8);
    // delay(200);
    // driveForward(100);
    // timerForward(50, 100);
    Move(-15, 140, 180);
    // delay(100);
    // PIDGyroTurnDegLookAt(-132, 135, 0);
    // delay(200);
    shoot(V2(-130, 135));
    // delay(100);
  }
  cout << "Second Basket :: " << Brain.timer(msec) - startTime << endl;

  intake(100);
  setShootTarget(friendlyHighGoal);
  {
    liftIntake(true);
    driveRotate(100);
    delay(100);
    intake(100);
    Move(28, 93.5, -47.5, 1, 1, CoR_To_Intake, moveType_LOCK);
    // delay(200);
    // PIDGyroTurnDegLookAt(30, 90, 0, 4, 50);
    // timerForward(80, 250);
    // rageIntake();
    delay(100);
    liftIntake(false);
    autoTrigger = true;
    delay(1200);
    shoot(V2(-130, 130));
    // delay(50);
  }
  cout << "Half time :: " << Brain.timer(msec) - startTime << endl;

  {
    liftIntake(true);
    setShootTarget(scoringHighGoal);
    intake(100);
    Move(91, 93, 0, 1, 1, CoR_To_Intake, moveType_LOCK);
    delay(100);
    // rageIntake();
    liftIntake(false);
    autoTrigger = true;
    delay(1200);
    //  timerForward(15,200);
    shoot(V2(132, -132));
    // delay(50);
    // while (1) {vexDelay(20);}
  }
  cout << "first stack" << Brain.timer(msec) - startTime << endl;

  setShootTarget(scoringHighGoal);
  {
    intake(100);
    // driveRotate(-100);
    // delay(100);
    Move(60, 60, 225, 0, 1, CoR_To_Intake);
    Move(100, 100, 45, 0, 1, CoR_To_Intake);
    Move(165, 160, 45, 0, 1, CoR_To_Intake);
    // driveForward(100);
    // delay(200);
    // // timerForward(100, 200);
    // // timerForward(-100, 300);
    // driveForward(-100);
    // delay(500);
    // intake(0);
    driveForward(-100);
    delay(400);
    // intake(0);
    Move(165, 125, 0, 1, 1, CoR_To_Intake);
    // intake(0);
    timerForward(80, 400);
    driveForward(20);
    encoderIndex(100, 200);
    intake(100);
    timerForward(-100, 200);
    // driveHorizontal(-100);
    // delay(300);
    Move(125, 162, 90, 1, 1, CoR_To_Intake);
    timerForward(80, 300);
    driveForward(20);
    encoderIndex(100, 100);
    // driveHorizontal(-85);
    // delay(200);
    // timerForward(-100, 200);
    driveForward(-100);
    delay(200);
  }
  cout << "roller and corner" << Brain.timer(msec) - startTime << endl;

  {
    // cout<<1<<endl;
    setShootTarget(friendlyHighGoal);
    intake(100);
    // focusBlue;
    // cout<<2<<endl;
    Move(39, 108, 170);
    // Move(-65,-59,100);
    shoot(V2(-130, 130));
    // delay(100);
  }
  cout << "corner disks + 1" << Brain.timer(msec) - startTime << endl;

  {
    driveRotate(-100);
    vexDelay(100);
    intake(100);
    setShootTarget(scoringHighGoal);
    Move(40, 60, 270, 0, 1, CoR_To_Intake);
    Move(41, -45, 270, 0, 1, CoR_To_Intake);
    Move(42, -15, -45, 0, 1, CoR_To_Intake);
    // Move(-15,15,180);
    Move(65, 0, 0, 0, 1, CoR_To_Intake);
    Move(160, -20, -80);
    shoot(V2(140, -135));
    // delay(50);
  }
  cout << "first drift" << Brain.timer(msec) - startTime << endl;
  setShootTarget(scoringHighGoal, 10);
  {
    intake(100);
    Move(132, -40, -130, 0, 1, CoR_To_Intake);
    Move(30, -42, -125, 1, 0.6, CoR_To_Intake);
    // delay(500);
    // lookAt(V2(-130, 130));
    // timerForward(50, 100);
    shoot(V2(135, -135));
    // delay(100);
  }
  cout << "first basket" << Brain.timer(msec) - startTime << endl;
  // delay(100);
  // intake(100);
  // Move(36, -40, -20);
  // Move(39, -125, -30, 0.8);
  // Move(30, -45, -45);
  // degarounded = false; punchPrep = true;
  // PIDGyroTurnDegLookAt(132, -135, 0);
  // while (!discorrect)
  // {
  //    delay(20);
  // }
  // hitting = true;
  // punlocking = true;
  // while (hitting)
  // {
  //    delay(10);
  // }
  // vexDelay(500);
  // Move(92, 28, 50);
  // Move(115, 105, 180);
  // degarounded = false;
  // punchPrep = true;
  // PIDGyroTurnDegLookAt(-132, 137, 0);
  // while (hitting || punchPrep)
  // {
  //    delay(10);
  // }
  autoTrigger = false;
  triggerDisTarget = 500;
  Move(-110, -100, 45, 0);
  // PIDGyroTurnDegLookAt(2000, 2000, 0);

  LF.stop(hold);
  LB.stop(hold);
  RF.stop(hold);
  RB.stop(hold);
  expand(1);

  cout << "Full time ::" << Brain.timer(msec) - startTime << endl;
  return 1;
}

#endif