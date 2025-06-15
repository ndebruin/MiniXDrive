#include <Arduino.h>
#include "AutonRunner.h"

AutonRunner::AutonRunner(Drivetrain* Drivetrain, PoseEstimator* Pose, State* RobotState):drivetrain(Drivetrain), pose(Pose), robotState(RobotState)
{ }


uint8_t AutonRunner::begin(UpdateFunction backgroundFunction){
    updateFunction = backgroundFunction;

    return 0;
}


uint8_t AutonRunner::update(){
    if(robotState->getRobotMode() == AUTO_MODE){
        // updateFunction(); // will run what we were given in setup
        return 1;
    }


    return 0;
}

void AutonRunner::execute(){
    switch (robotState->selectedAuton){

    }
}

void AutonRunner::delayWithoutBlocking(int32_t timeMS){
    startTime = millis();
    while((millis() - startTime) < timeMS){
        updateFunction();
    }
    startTime = millis();
}
