#include <Arduino.h>
#include <Alfredo_NoU2.h>
#include "Drivetrain.h"

Drivetrain::Drivetrain(NoU_Drivetrain* NoUDrivetrain, PoseEstimator* poseEstimator, State* RobotState) : nouDrivetrain(NoUDrivetrain), pose(poseEstimator), robotState(RobotState)
{ }

uint8_t Drivetrain::begin()
{
    // setup intake curve values
    nouDrivetrain->setMinimumOutput(kS);
    nouDrivetrain->setInputExponent(driveExp);

    return 0;
}

uint8_t Drivetrain::update()
{
    // safety measure
    if(!robotState->isEnabled()){
        stop();
    }

    if(driveMode == THETA_ONLY){
        thetaError = (desiredTheta - pose->getCurrentGlobalPose().yaw);

        // suck it we're writing our own feedforward + P controller
        angularZCommand = ((thetaError/fabs(thetaError))*angZ_kS + (thetaError * angZ_kP)) * thetaPower;

        // if we're in this mode we just want to do this continously until told otherwise, therefore there is no termination clause
    }

    if(driveMode == FULL_AUTO){
        
        xError = (desiredX - pose->getCurrentGlobalPose().x);
        yError = (desiredY - pose->getCurrentGlobalPose().y);
        thetaError = (desiredTheta - pose->getCurrentGlobalPose().yaw);

        Serial.println("Error:" + String(xError) + "x" + String(yError) + "y" + String(thetaError) + "t");

        // suck it we're writing our own feedforward + P controller
        linearXCommand = -((xError/fabs(xError))*linX_kS + (xError * linX_kP)) * xPower;
        linearYCommand = -((yError/fabs(yError))*linY_kS + (yError * linY_kP)) * yPower;
        angularZCommand = ((thetaError/fabs(thetaError))*angZ_kS + (thetaError * angZ_kP)) * thetaPower;

        Serial.println("Command:" + String(linearXCommand) + "x" + String(linearYCommand) + "y" + String(angularZCommand) + "t");

        // just to be safe
        setFieldOriented(FIELD_ORIENTED);

        // if we're enabled and in an auto Mode (only current time we use the full pose controller), use the control values
        if(robotState->isEnabled() && (robotState->getRobotMode() == AUTO_MODE)){
            drive(linearXCommand, linearYCommand, angularZCommand); // drive with our commands
        }

        // this defines the end case
        if(xPower == 0.0 && yPower == 0.0 && thetaPower == 0.0){
            Serial.println("testing");
            xPower = 1.0;
            yPower = 1.0;
            thetaPower = 1.0;
            if(fabs(thetaError) < theta_AcceptableError && fabs(yError) < y_AcceptableError && fabs(xError) < x_AcceptableError){ // if we're actually done
                Serial.println("STOPPING");
                driveMode = FULL_TELEOP;
                stop();
                reachedGoal = true;
                return 1;
            }
        }
        
    }

    if(fabs(thetaError) < theta_AcceptableError
        )
        {
            correctCounterTheta++;
            
            if(correctCounterTheta > correctCount){
                Serial.println("DONE YAW");
                thetaPower = 0.0;
                correctCounterTheta = 0;
            }
            else{
                thetaPower = 1.0 - ((float)correctCounterTheta / (float)correctCount); // this will reduce the output power as a function of the number of times we are within the range
                // this should dampen the osillations a bit (kinda a psuedo D term)
            }
    }
    else if(fabs(thetaError) > theta_AcceptableError){
        correctCounterTheta = 0;
    }

    if(fabs(xError) < x_AcceptableError)
        {
            correctCounterX++;
            
            if(correctCounterX > correctCount){
                Serial.println("DONE X");
                xPower = 0.0;
                correctCounterX = 0;
            }
    }
    else if(fabs(xError) > x_AcceptableError){
        correctCounterX = 0;
    }

    if(fabs(yError) < y_AcceptableError)
        {
            correctCounterY++;
            
            if(correctCounterY > correctCount){
                Serial.println("DONE Y");
                yPower = 0.0;
                correctCounterY = 0;
            }
    }
    else if(fabs(yError) > y_AcceptableError){
        correctCounterY = 0;
    }    

    return 0;
}

void Drivetrain::teleopDrive(float linearX, float linearY, float angularZ)
{
    if(driveMode == FULL_AUTO){ // something has gone wrong (for now, this may change if we do drive-to-goal in the future)
        driveMode = FULL_TELEOP;
    }
    else if(driveMode == THETA_ONLY && angularZ != 0.0){ // there is manual heading data coming in, revert to manual control
        driveMode = FULL_TELEOP;
    }
    else if(driveMode == THETA_ONLY && angularZ == 0.0){
        // if the robot is doing heading control and there's no input from the driver
        // we want to use the command control value, not the drivers
        drive(linearX, linearY, angularZCommand); 
    }

    // normal mode
    if(driveMode == FULL_TELEOP){
        drive(linearX, linearY, angularZ);
    }
}

void Drivetrain::drive(float linearX, float linearY, float angularZ)
{
    if(fieldOriented)
    {
        float temp = linearX* cos(pose->getYaw()*DEG_TO_RAD) + -linearY* sin(pose->getYaw()*DEG_TO_RAD); // this is the systems of equations form of a 2x2 rotation matrix
        linearY = linearX * sin(pose->getYaw()*DEG_TO_RAD) + linearY* cos(pose->getYaw()*DEG_TO_RAD); // this is also the commonly used field oriented drive equations
        linearX = temp;
    }
    if(robotState->isEnabled())
    {
        nouDrivetrain->holonomicDrive(linearX, linearY, angularZ);
    }

    return;
}

void Drivetrain::drive(float linearX, float linearY, float angularZ, bool fieldOrientedEnabled)
{
    fieldOriented = fieldOrientedEnabled;
    drive(linearX, linearY, angularZ);

    return;
}

void Drivetrain::stop(){
    nouDrivetrain->holonomicDrive(0, 0, 0);

    return;
}

void Drivetrain::setPose(Pose pose){
    desiredX = pose.x;
    desiredY = pose.y;
    desiredTheta = pose.yaw;
    driveMode = FULL_AUTO;
    setFieldOriented(FIELD_ORIENTED); 
    // we actually want field oriented b.c. we're doing our path following in the inertial(field) frame
    // therefore field oriented actually controls how we would like.
    cancelWhenInRange = true;

    xPower = 1.0;
    yPower = 1.0;
    thetaPower = 1.0;
}

void Drivetrain::setTheta(float theta){
    desiredTheta = theta;
    driveMode = THETA_ONLY;
    cancelWhenInRange = false; // if we're doing heading control, we want to do it constantly

    thetaPower = 1.0;
}