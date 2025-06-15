#ifndef AUTONRUNNER_h
#define AUTONRUNNER_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "Constants.h"

#include "Drivetrain.h"
#include "PoseEstimator.h"
#include "State.h"


typedef void (*UpdateFunction)();

class AutonRunner
{
    public:
        AutonRunner(Drivetrain* Drivetrain, PoseEstimator* Pose, State* RobotState);

        uint8_t begin(UpdateFunction backgroundFunction);
        uint8_t update();
        void execute();

    private:
        void delayWithoutBlocking(int32_t millis);
        uint32_t startTime;

        UpdateFunction updateFunction;
        Drivetrain* drivetrain;
        PoseEstimator* pose;
        State* robotState;
};

#endif // AUTONRUNNER_h