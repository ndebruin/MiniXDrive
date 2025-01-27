#include <Arduino.h>

// Alfredo Stuff
#include <Alfredo_NoU2.h>
// #include <BluetoothSerial.h>
// #include <AlfredoConnect.h>
#include <PestoLink-Receive.h>

// Subsystems
#include "Drivetrain.h"

// other custom code
#include "State.h"
#include "PoseEstimator.h"
#include "AutonRunner.h"

#include "Constants.h"

////////////////////////////////////////////////////////////////////// Hardware Declarations //////////////////////////////////////////////////////////////////////

// BluetoothSerial SerialBluetooth; // bluetooth link

State state;

PoseEstimator pose(&Serial, baudRate, &state); // we're just running the coproc serial over the normal serial bus.
                                       // this kinda sucks bc it means that we need to disconnect it whenever we program, but oh well
                                       // can still use Serial.println() for normal computer info tho

// create our actual motors and servos

NoU_Motor frontLeftMotor(frontLeftMotorChannel);
NoU_Motor frontRightMotor(frontRightMotorChannel);
NoU_Motor backLeftMotor(backLeftMotorChannel);
NoU_Motor backRightMotor(backRightMotorChannel);

NoU_Drivetrain nou_drivetrain(&frontLeftMotor, &frontRightMotor, &backLeftMotor, &backRightMotor);

// create our subsystem objects

Drivetrain drivetrain = Drivetrain(&nou_drivetrain, &pose, &state);

// create our object definitions for fancy stuff (path following, autoaim, etc)

AutonRunner autonRunner = AutonRunner(&drivetrain, &pose, &state);

////////////////////////////////////////////////////////////////////// Function Declerations //////////////////////////////////////////////////////////////////////

void asyncUpdate();

double deadzone(double raw);
void constantButtons();
void runDrivetrain();

void updatePestoLink();

////////////////////////////////////////////////////////////////////// Global Variables //////////////////////////////////////////////////////////////////////

bool justExecuted = false;
bool justIntake = false;

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////

void setup() 
{
  // start up bluetooth link for alfredoconnect
  // SerialBluetooth.begin(robotName);
  // AlfredoConnect.begin(SerialBluetooth);

  // Serial.begin(9600);

  frontRightMotor.setInverted(true);
  backRightMotor.setInverted(true);

  PestoLink.begin(robotName);

  // start RSL
  RSL::initialize();

  // start subsystems
  drivetrain.begin();

  // start our pose estimator
  pose.begin();
  
  // start advanced controllers
  autonRunner.begin(asyncUpdate);
}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

void loop() 
{
  asyncUpdate(); // updates all the things that need to be updated every loop regardless of anything else

  // Serial.println(String(pose.getCurrentGlobalPose().x) + "x" + String(pose.getCurrentGlobalPose().y) + "y" + String(pose.getCurrentGlobalPose().yaw) + "t");

  if(state.getRobotMode() == TELEOP_MODE && state.isEnabled()){ // if in teleop
    // handle drivetrain
    runDrivetrain();
  }

}

////////////////////////////////////////////////////////////////////// Function Definitions //////////////////////////////////////////////////////////////////////

void asyncUpdate(){
  // let subsystems code update
  drivetrain.update();

  // update our pose
  pose.update();

  // let advanced controllers update
  autonRunner.update();

  // update from driver station
  if(!PestoLink.update()){
    state.setEnable(DISABLE); // disable if we disconnect
  }

  // rsl code
  RSL::update();

  if(state.isEnabled()){
    RSL::setState(RSL_ENABLED);
  }
  else if(!state.isEnabled()){
    RSL::setState(RSL_OFF);
  }

  // update pestolink telem
  updatePestoLink();
  constantButtons();
}

void updatePestoLink(){
  String telemString;

  telemString.concat(' ');
  telemString.concat(String(pose.getYaw()));

  char* telem = (char*)telemString.c_str();
  // 8/8 characters

   // update pestolink telem
  if(state.isEnabled()){
    PestoLink.print(telem, "00FF00");
  }
  else{
    PestoLink.print(telem, "FF0000");
  }
  return;
}

double deadzone(double rawJoy){
  if(fabs(rawJoy) < deadzoneValue){
    return 0.0;
  }
  return rawJoy;
}

void constantButtons(){
  // enable / disable logic
  if(PestoLink.buttonHeld(buttonDisable)){
    state.setEnable(DISABLE);
  }
  else if(PestoLink.buttonHeld(buttonEnable)){
    state.setEnable(ENABLE);
  }


  if(PestoLink.buttonHeld(buttonZeroYaw)){ // reset IMU yaw
    pose.zeroYaw();
  }

  // holy crap this is a really bad overload
  if(PestoLink.buttonHeld(buttonTeleopMode)){
    state.setTeleopMode();
  }

  // enable / disable field oriented driving
  if(PestoLink.buttonHeld(buttonEnableFieldOriented)){ 
    drivetrain.setFieldOriented(FIELD_ORIENTED);
  }
  else if(PestoLink.buttonHeld(buttonDisableFieldOriented)){
    drivetrain.setFieldOriented(ROBOT_ORIENTED);
  }
}

void runDrivetrain(){
  float linearX = deadzone(PestoLink.getAxis(axisLinX));
  float linearY = -deadzone(PestoLink.getAxis(axisLinY));
  float angularZ = deadzone(PestoLink.getAxis(axisAngZ)) * 0.5;
  
  drivetrain.teleopDrive(linearX, linearY, angularZ);

  return;
}

