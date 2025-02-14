#pragma once
#include "vex.h"
#include <iostream>
#define printToConsole(text) std::cout<<text<<std::endl;

vex::brain Brain = vex::brain();
vex::controller Controller1(vex::primary);
vex::motor l1(vex::PORT10, vex::ratio6_1, true);
vex::motor l2(vex::PORT15, vex::ratio6_1, true);
vex::motor l3(vex::PORT19, vex::ratio6_1, true);
vex::motor r1(vex::PORT5, vex::ratio6_1, false);
vex::motor r2(vex::PORT8, vex::ratio6_1, false);
vex::motor r3(vex::PORT16, vex::ratio6_1, false);
vex::motor intake1(vex::PORT14, vex::ratio6_1, false);
vex::motor intake2(vex::PORT20, vex::ratio6_1, false);
vex::motor_group Left(l1,l2,l3);
vex::motor_group Right(r1,r2,r3);
vex::motor_group intake(intake1, intake2);
vex::digital_out clampPneumatics(Brain.ThreeWirePort.A);
vex::inertial InertialSensor(vex::PORT13);