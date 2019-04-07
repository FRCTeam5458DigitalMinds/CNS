/*
  2019 - CNS 2.0
*/

#include <string>
#include <sstream>
#include <Robot.h>
#include <WPILib.h>
#include <stdlib.h>
#include <iostream>
#include <frc/Timer.h>
#include <TimedRobot.h>
#include <frc/Solenoid.h>
#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/smartdashboard/SmartDashboard.h>


/* Declarations */


/* Digital Inputs / Electrical components */

// PDP
frc::PowerDistributionPanel pdp{0};
// Gyro
frc::ADXRS450_Gyro Gyro{}; 
// Straightens out the bot
float signed_square(float x){
  return x * fabsf(x);
}
float LastSumAngle;
float turnFact = 0.9;
// Joystick & Racewheel
frc::Joystick JoyAccel1{0}, Xbox{1}, RaceWheel{2};


/* Motors */

// Right Side Drive Motors
// Right Side
WPI_TalonSRX RightFront{0};
WPI_TalonSRX RightBack{0};
// Left Side
WPI_TalonSRX LeftFront{0};
WPI_TalonSRX LeftBack{0};
//Intake Motors
WPI_TalonSRX RightIntake{0};
WPI_TalonSRX LeftIntake{0};
WPI_TalonSRX PopUp{0};
WPI_TalonSRX Pivot{0};
//Conveyer Motors
//Right Side
WPI_TalonSRX RightFrontConveyer{0};
WPI_TalonSRX RightMidConveyer{0};
WPI_TalonSRX RightBackConveyer{0};
//Left Side
WPI_TalonSRX LeftFrontConveyer{0};
WPI_TalonSRX LeftMidConveyer{0};
WPI_TalonSRX LeftBackConveyer{0};

/* Pneumatics */

//Intake
frc::Solenoid CubeIntake{0};
bool CubeButton = false;
//Ramps
frc::Solenoid RampDeploy{0};
bool RampButton = false;
frc::Solenoid LeftRamp{0};
bool LeftRampButton = false;
frc::Solenoid RightRamp{0};
bool RightRampButton = false;


bool beRunning = false;


void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Right side of the drive motors
  RightFront.SetInverted(true);
  RightBack.SetInverted(true);

  RampDeploy.Set(false);
  CubeIntake.Set(false);
  LeftRamp.Set(false);
  RightRamp.Set(false);


}

void Robot::TeleopPeriodic() {}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::AutonomousInit() {}


/*Called on every robot packet, no matter what mode*/
void Robot::RobotPeriodic() {

  //Gets axis for each controller (Driving/Operating)
  double JoyY = -JoyAccel1.GetY();
  double WheelX = RaceWheel.GetX();

  //Power get's cut from one side of the bot to straighten out when driving straight
  float sumAngle = Gyro.GetAngle();
  float derivAngle = sumAngle - LastSumAngle;
  float correctionAngle = (sumAngle * 0.00) + (derivAngle *0.00);
  //                                    ^subject to change...

  //Ramp bois

  //Ramps down
  if (JoyAccel1.GetRawButton(7)) {
    if(!RampButton) {
      RampDeploy.Set(!RampDeploy.Get());
      RampButton = true;
    }
  }
  else {
    RampButton = false;
  }
  //Left Ramp lift
  if (JoyAccel1.GetRawButton(11)) {
    if(!LeftRampButton) {
      LeftRamp.Set(!LeftRamp.Get());
      LeftRampButton = true;
    }
  }
  else {
    LeftRampButton = false;
  }
  // Right Ramp lift
  if (JoyAccel1.GetRawButton(10)) {
    if(!RightRampButton) {
      RightRamp.Set(!RightRamp.Get());
      RightRampButton = true;
    }
  }
  else {
    RightRampButton = false;
  }


 //Intakes out
  if (Xbox.GetRawButton(8)) {
    if(!CubeButton) {
      CubeIntake.Set(!CubeIntake.Get());
      CubeButton = true;
    }
  }
  else {
    CubeButton = false;
  }
  // Intake cube
  if (Xbox.GetRawButton(3)) {

    RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
    LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
    PopUp.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
  } 
  else {
    RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  }

  // Spit cube
  if (Xbox.GetRawButton(1)) {

    RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    PopUp.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
  }
  else {
    RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  }

//Shoots the cube
if (Xbox.GetRawButton(5)) {

  RightFrontConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
  RightMidConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
  RightBackConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
  LeftFrontConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
  LeftMidConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
  LeftBackConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);

}
//Shoots Through the front
if (Xbox.GetRawButton(1)) {

  RightFrontConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
  RightMidConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
  RightBackConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
  LeftFrontConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
  LeftBackConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
  PopUp.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
  LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2);
  RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.2);
}
else {
  RightFrontConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  RightMidConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  RightBackConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  LeftFrontConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  LeftBackConveyer.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  PopUp.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  LeftIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
  RightIntake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
}
  //Drive Code for CNS and modified for Axon
  //Button 5 on the wheel activates point turning
  if (RaceWheel.GetRawButton(5)) {
    RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, WheelX);
    LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -WheelX);
    Gyro.Reset();
  } 
  else {
    //Code for regular turning
    if ((WheelX < -0.01 || WheelX > 0.01) && (JoyY > 0.06 || JoyY < -0.06)) {
      //              ^ You set the dead zone for the wheel and the joysticks
      RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + turnFact*(WheelX));
      RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + turnFact*(WheelX));
      LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - turnFact*(WheelX));
      LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -JoyY - turnFact*(WheelX));
      Gyro.Reset();
    }
    //Code for driving straight
    else if ((JoyY > 0.1|| JoyY < -0.1)) {
      //              ^ You set the dead zone for the joystick
      RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - correctionAngle);
      RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY - correctionAngle);
      LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + correctionAngle);
      LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -JoyY + correctionAngle);
    } 
    else {
      if(!beRunning) {
        // Dont spin any drive train motors if the driver is not doing anything
        RightFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        RightBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        LeftFront.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        LeftBack.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      }
    }
  }
  
  /* Straightens out bot here when driving straight */
  LastSumAngle = sumAngle;

}

/* Called every robot packet in testing mode */
void Robot::TestPeriodic() {}

/* Starts the bot */
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
