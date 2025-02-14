#include "cmath"
#include "vex.h"
#include "motors.h"
#include "turnPID.h"
#include <iostream>


// Try: Increasing kD, make integral = 0 when changes signs and increase end error value

    turnPID::turnPID() {
    }

    void turnPID::reset()
    {
        error = 0;
        prev = 0;
        i = 0;
        // Left.resetPosition();
        // Right.resetPosition();
    }


    void turnPID::tpUpdate()
    {
        position = InertialSensor.heading();


        error = target - position;

        if (error > 180) {
            //printToConsole(error << " is greater than 180");
            error -= 360;
        } else if (error < -180) {
            //printToConsole(error << " is less than -180");
            error += 360;
        }

        
        //std::cout<<abs(target - position)<<std::endl;

        // printToConsole(error); // MOST RECENT DEBUG OUTPUT


        // printToConsole((kp * error) + (ki * i) + (kd * d));
        // std::cout<<error<<std::endl;

        Brain.Screen.clearLine();
        Brain.Screen.print(error);

        i = i + error + (prev - error) / 2.0;
        d = error - prev;
        prev = error;
        if (error / fabs(error) != prev / fabs(prev))
        {
            i = 0;
            
        }

        if (fabs(i) >= 100)
        {
            i = (i / fabs(i)) * 100;
        }
        

    }


    void turnPID::stopTurnPID() {
        Left.stop(vex::brake);
        Right.stop(vex::brake);
        Left.setStopping(vex::coast);
        Right.setStopping(vex::coast);
        printToConsole("turn PID stopped");
    }

    void turnPID::runTurnPID(double targetVal/*, vex::motor_group Left, vex::motor_group Right*/)
    {
        int time = 0;
        // std::cout<<position<<std::endl;
        position = 0;
        //InertialSensor.resetHeading();
        target = targetVal;
        // printToConsole("Target: " << target);
        // printToConsole("Position: " << position);
        // printToConsole(target - position);

        bool fixPos = false;
        // std::cout<<position<<std::endl;
        // wait(1, vex::sec);
        if (target > 180) {
            target -= 360;
        } else if (target < -180) {
            target += 360;
        }
        if (target == 0) {
            InertialSensor.setHeading(InertialSensor.heading() + 90, vex::deg);
            target += 90;
            fixPos = true;
        }
        
        while (fabs(target - position) > 4) {
            tpUpdate();
            // std::cout<<"h"<<std::endl;
            // std::cout<<position<<std::endl;
            //spinAll(true, (kp * error) + (ki * i) + (kd * d));
            Left.spin(vex::forward, ((kp * error) + (ki * i) + (kd * d)) * 0.3, vex::pct);
            Right.spin(vex::reverse, ((kp * error) + (ki * i) + (kd * d)) * 0.3, vex::pct);
            // tpUpdate();

            if (position > 360) {
                position -= 360;
            } else if (position < -360) {
                position += 360;
            }
            
            time += 20;
            vex::wait(20, vex::msec);
            if (time >= 2000) {
                printToConsole("Time Limit Reached");
                stopTurnPID();
                break;
            }

            if (fabs((kp * error) + (ki * i) + (kd * d)) < 1) {
                printToConsole("Too slow; saving time");
                stopTurnPID();
                break;
            }

            if (fabs(target - position) <= 16) { //Trying to slow it down
                error /= 2;
            }
        } 
        if (fixPos) {
            InertialSensor.setHeading(InertialSensor.heading() - 90, vex::deg);
        }
        Left.stop(vex::brake);
        Right.stop(vex::brake);
        std::cout<<"done :)"<<std::endl;

        vex::wait(20, vex::msec);
        
    }

    /*void turnPID::shake(double seconds)
    {
        if (seconds == 0) seconds = 1; 
        double startingPos = InertialSensor.heading();
        double timeLimit = seconds;
        int time = 0;
        // std::cout<<position<<std::endl;
        position = 0;
        //InertialSensor.resetHeading();
        // int targetMod = 8;
        // target = InertialSensor.heading();
        target = 8;
        // printToConsole("Target: " << target);
        // printToConsole("Position: " << position);
        // printToConsole(target - position);
        // std::cout<<position<<std::endl;
        // wait(1, vex::sec);
        if (target > 180) {
            target -= 360;
        } else if (target < -180) {
            target += 360;
        }
        
        while (time < timeLimit * 1000) {
            // target = InertialSensor.heading() + targetMod;
            tpUpdate();
            // std::cout<<"h"<<std::endl;
            // std::cout<<position<<std::endl;
            //spinAll(true, (kp * error) + (ki * i) + (kd * d));
            Left.spin(vex::forward, ((kp * error) + (ki * i) + (kd * d)) * 24, vex::pct);
            Right.spin(vex::reverse, ((kp * error) + (ki * i) + (kd * d)) * 24, vex::pct);
            // tpUpdate();
            // printToConsole(position - startingPos);

            if (fabs(position) >= fabs(target) && (signof(position) == signof(target))) {
                // Left.stop(vex::brake);
                // Right.stop(vex::brake);
                printToConsole("Reverse");
                target *= -1;
                reset();
            }

            if (position > 360) {
                position -= 360;
            } else if (position < -360) {
                position += 360;
            }
            
            time += 20;
            vex::wait(20, vex::msec);
            if (time >= timeLimit * 1000) {
                printToConsole("[Shaking] Time Limit Reached");
                stopTurnPID();
                break;
            }   

        } 
        Left.stop(vex::brake);
        Right.stop(vex::brake);
        std::cout<<"done shaking"<<std::endl;

        runTurnPID(startingPos);

        vex::wait(20, vex::msec);
        
    }*/