#ifndef POSEESTIMATOR_h
#define POSEESTIMATOR_h

#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "coProcCom.h"
#include "Constants.h"
#include "State.h"

struct Pose{
    double x;
    double y;
    double yaw;
};

class PoseEstimator
{
    public:
        PoseEstimator(HardwareSerial *SerialPort, uint BaudRate, uint8_t rxPin, uint8_t txPin, State* state);
        PoseEstimator(HardwareSerial *SerialPort, uint BaudRate, State* state);

        uint8_t begin();
        uint8_t update();

        double getYaw(){
            return currentGlobalPose.yaw;
        }

        void zeroYaw(){
            yawOffset = rawYaw;
            if(robotState->getAlliance() == BLUE){
                yawOffset -= 180.0;
            }
        }
        
        Pose getCurrentGlobalPose() { return currentGlobalPose; }

        Pose GlobaltoRobotPose(Pose globalPose);
        Pose RobottoGlobalPose(Pose robotPose);

        Pose subtractPose(Pose Pose1, Pose Pose2);

        float lengthOfPose(Pose pose);

        float thetaOfPose(Pose pose);

        void resetPose(Pose pose);

        void zeroPose();

    private:
        State* robotState;
        HardwareSerial *serial;
        uint baud;
        uint8_t tx, rx = 0;

        CoProcStructRX rxDataStruct;

        Pose currentGlobalPose;
        double rawYaw;

        double yawOffset;
        
        void supplementPose(Pose *mainPose, Pose addPose);

        bool coProcReceive(CoProcStructRX* table);
        bool updateFromCoProc();
        
 
};

#endif // POSEESTIMATOR_h