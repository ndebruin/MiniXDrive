#ifndef Drivetrain_h
#define Drivetrain_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "PoseEstimator.h"


#include "State.h"

#define FIELD_ORIENTED true
#define ROBOT_ORIENTED false

#define FULL_TELEOP 0
#define THETA_ONLY 1
#define FULL_AUTO 2

class Drivetrain
{
    public:
        Drivetrain(NoU_Drivetrain* NoUDrivetrain, PoseEstimator* poseEstimator, State* robotState);
        uint8_t begin();
        uint8_t update();

        bool getFieldOriented(){ return fieldOriented; }

        void setFieldOriented(bool FieldOriented){ fieldOriented = FieldOriented; }

        void teleopDrive(float linearX, float linearY, float angularZ);

        // will either be field oriented or not depending on the seperately set field (setFieldOriented(bool))
        void drive(float linearX, float linearY, float angularZ);

        // will either be field oriented or not depending on the passed boolean
        void drive(float linearX, float linearY, float angularZ, bool fieldOrientedEnabled); 

        void stop();

        void setPose(Pose pose);
        void setTheta(float theta);
        
        bool reachedGoal = false;

    private:
        NoU_Drivetrain* nouDrivetrain;

        PoseEstimator* pose;
        State* robotState;

        bool fieldOriented = false;

        int correctCounterTheta, correctCounterX, correctCounterY;

        float thetaPower, xPower, yPower;

        // 0 - full teleop
        // 1 - auto theta only
        // 2 - full auto
        uint8_t driveMode;

        float desiredX, xError, linearXCommand;
        float desiredY, yError, linearYCommand;
        float desiredTheta, thetaError, angularZCommand;

        bool cancelWhenInRange;
 
};

#endif