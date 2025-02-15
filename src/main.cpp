/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       orang                                                     */
/*    Created:      2/14/2025, 12:20:45 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "motors.h"

using namespace vex;

// A global instance of competition
competition Competition;
// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

double kp1 = 2;
double ki1 = 0;
double kd1 = 0;
double time2 = 0;
double error;
bool PIDBool;
double i;
double prev_error;
double d;
double p;
double angle;
bool wSMBool = false;

bool doinker = false;
bool clamped = false;

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void PID(double target, double timeLimit) {
  PIDBool = true;
  time2 = time2+ 10;
  i = 0; 
  d=0;
  while (PIDBool) {
    time2 = time2+ 10;
    error = (fabs(l2.position(turns)*3.25*2*M_PI)) - target;
    i = i + error; 
    if (error == 0) {
     i = 0;
    }
    
    /*if (time2 >= (timeLimit *1000)) {
      leftDrive.stop(coast);
      rightDrive.stop(coast);
      PIDBool = false;
      break;
    }*/
    Left.spin(forward,((kp1*error)+ (ki1*i) + (kd1*d)), pct);
    Left.spin(forward,((kp1*error)+ (ki1*i) + (kd1*d)), pct);
    prev_error = error;
    d = error-prev_error;
    wait(10,msec);
  }
  time2 = 0;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}
void clampMacro() {
  if (clamped) {
    clampPneumatics.set (false);
    clamped = false;
  } else {
    clampPneumatics.set(true);
    clamped = true;
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  double slow = 1;
  bool pneumaticsBool = false;
  bool pressingBool = false;
  while (1) {
    if (Controller1.ButtonUp.pressing()){
      slow = 0.4;
    } else {
      slow = 1;
    }

    Left.spin(forward, (Controller1.Axis3.position() + Controller1.Axis1.position()) * 0.6, pct);
    Right.spin(forward, (Controller1.Axis3.position() - Controller1.Axis1.position()) * 0.6, pct);

    if (Controller1.ButtonR1.pressing()) {
      intake.spin(forward, 100, pct);
    } else if (Controller1.ButtonR2.pressing()) {
      intake.spin(reverse, 100, pct);
    } else {
      intake.stop(brake);
    }
    
    if (Controller1.ButtonA.pressing()) {
      if (pressingBool == false) {
        pneumaticsBool = !pneumaticsBool;
        clampPneumatics.set(pneumaticsBool);
        printToConsole(pneumaticsBool);
      }
      pressingBool = true;
    } else {
      pressingBool = false;
    }
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
