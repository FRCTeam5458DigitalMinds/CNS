#include <WPILib.h>
#include <RobotDrive.h>
#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include <iostream>
#include <Solenoid.h>
#include <DigitalInput.h>
#include <math.h>
#include "ctre/Phoenix.h"
#include <ADXRS450_Gyro.h>
#include <Encoder.h>
#include <cmath>
#include <time.h>
#include "Spark.h"
#include "networktables/NetworkTable.h"


class Robot : public frc::IterativeRobot{
//Declarations
	RobotDrive *DriveTrain;
	Joystick *JoyAccel, *RaceWheel, *JoyAccel2;
	PowerDistributionPanel *pdp;
	ADXRS450_Gyro *gyro;
	Encoder *leftEnc = new Encoder(0, 1);
	Encoder *rightEnc = new Encoder(2, 3);
	Solenoid *intake, *rampDeploy, *leftRamp, *rightRamp;

	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");

	float yInput, xInput, gyroFact, turnFact, spawn, lastSumAngle, step, side;
	float power = 0;
	float anglePower = 1;

	float leftLeftDist[1] = {140};
	float rightRightDist[1] = {140};

	float rightLeftDist[1] = {140};
	float leftRightDist[1] = {140};

	float middleRightDist[7] = {20, 47, 18, 35, 20, 0, 35};
	float middleLeftDist[7] = {20, 47, 18, 35, 20, 0, 35};

	float rightRightAngle[1] = {75};
	float leftLeftAngle[1] = {75};

	float middleRightAngle[4] = {60, 60, 125, 125};
	float middleLeftAngle[4] = {60, 60, 123, 123};

	float straightDistance;

	bool countingTime = false;
	bool EmergencyIntake = false;
	time_t Emergency;

	bool pivotBack = true;
	bool intakeWasPressed = false;
	bool liftWasPressed = false;

	std::string gameData;

private:
	DigitalInput *switch1, *switch2;
	DigitalInput *blockSensor1;
	DigitalInput *blockSensor2;

	VictorSPX *intakeLeft, *intakeRight;
	VictorSPX *rightBack, *leftFront;
	TalonSRX *rightFront, *leftBack;
	VictorSPX *popUp, *leftBackConveyor;
	TalonSRX *frontRightConveyor, *frontLeftConveyor;
	TalonSRX *rightMidConveyor, *leftMidConveyor;
	TalonSRX *rightBackConveyor, *pivot;

public:
	void RobotInit() override
{

		pdp = new PowerDistributionPanel(0);

		gyro = new ADXRS450_Gyro();
		gyro->Reset();

		blockSensor1 = new DigitalInput(4);
		blockSensor2 = new DigitalInput(9);

		switch1 = new DigitalInput(6);
		switch2 = new DigitalInput(8);

		JoyAccel = new Joystick(0);
		JoyAccel2 = new Joystick(2);
		RaceWheel = new Joystick(1);

		intake = new Solenoid(0);
		rampDeploy = new Solenoid(2);
		leftRamp = new Solenoid(4);
		rightRamp = new Solenoid(5);
		intake->Set(false);
		rampDeploy->Set(false);
		leftRamp->Set(false);
		rightRamp->Set(false);

		rightFront = new TalonSRX(0);
		leftFront = new VictorSPX(14);
		rightBack = new VictorSPX(1);
		leftBack = new TalonSRX(15);

		pivot = new TalonSRX(13);
		intakeLeft = new VictorSPX(9);
		intakeRight = new VictorSPX(6);
		popUp = new VictorSPX(3);
		popUp->SetInverted(true);
		intakeLeft->SetInverted(true);
		intakeRight->SetInverted(true);

		frontLeftConveyor = new TalonSRX(10);
		frontRightConveyor = new TalonSRX(2);
		rightMidConveyor = new TalonSRX(7);
		leftMidConveyor = new TalonSRX(8);
		rightBackConveyor = new TalonSRX(5);
		leftBackConveyor = new VictorSPX(12);
		frontRightConveyor->SetInverted(false);
		frontLeftConveyor->SetInverted(true);
		rightMidConveyor->SetInverted(true);
		leftMidConveyor->SetInverted(true);

		leftEnc->SetDistancePerPulse(1.0 / 360.0 * 2.0 * M_PI * 3);
		leftEnc->Reset();
		rightEnc->SetDistancePerPulse(1.0 / 360.0 * 2.0 * M_PI * 3);
		rightEnc->Reset();

		pivotBack = true;

		step = 1;
		/* For Sides
		 * 0 = left
		 * 1 = middle
		 * 2 = right
		 */

		gyroFact = 0.1;
		turnFact = 0.9;
}

	/**
	 * This function is called once each time the robot enters Disabled
	 * mode.
	 * You can use it to reset any subsystem information you want to clear
	 * when
	 * the robot is disabled.
	 */
	void DisabledInit() override
{

}

	void DisabledPeriodic() override
{
		frc::Scheduler::GetInstance()->Run();

}

	/**
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to
	 * the
	 * chooser code above (like the commented example) or additional
	 * comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {

		leftEnc->SetDistancePerPulse(1.0 / 360.0 * 2.0 * M_PI * 3);
		rightEnc->SetDistancePerPulse(1.0 / 360.0 * 2.0 * M_PI * 3);
		gyro->Reset();
		leftEnc->Reset();
		rightEnc->Reset();
		step = 1;
		power = 0;
		anglePower = 1;

		intake->Set(false);

		pivotBack = true;

		gameData = "";

		if (!switch1->Get() && !switch2->Get()) side = 0;
		else if (switch1->Get() && !switch2->Get()) side = 1;
		else if (!switch1->Get() && switch2->Get()) side = 2;

	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();

		float sumAngle = gyro->GetAngle();
		float derivAngle = sumAngle - lastSumAngle;
		float correctionAngle = (sumAngle * 0.1) + (derivAngle * .2);

		if (leftEnc->GetDistance() == 0) straightDistance = fabs(rightEnc->GetDistance());
		else if (rightEnc->GetDistance() == 0) straightDistance = fabs(leftEnc->GetDistance());
		else straightDistance = (fabs(leftEnc->GetDistance()) + fabs(rightEnc->GetDistance()) / 2);

		//std::cout << "Right: " << rightEnc->GetDistance() << std::endl;
		//std::cout << "Left: " << leftEnc->GetDistance() << std::endl;
		//std::cout << "Angle Power: " << anglePower << std::endl;
		//std::cout << "gyro: " << gyro->GetAngle() << std::endl;
		//std::cout << "Step: " << step << std::endl;

		//Code for the sensors on the pivot that keep the block in place while driving
		if (blockSensor1->Get()) {
			frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.3);
			frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.3);
			rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
			leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
		}
		else if (!blockSensor1->Get()) {

			if (!blockSensor2->Get()) {
				frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
				frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
				rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			}
			else {
				frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			}
		}

		if (gameData.length() < 1) {
			gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		}
		else {
			if(gameData[0] == 'L') {
				if (side == 1) {
					if (step == 1) {
						if (straightDistance < middleLeftDist[0]) {
							autoBackward(correctionAngle, middleLeftDist[0]);
							pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);
						}
						else {
							autoStop();
							step = 2;
						}
					}
					else if (step == 2) {
						if (fabs(gyro->GetAngle()) < middleLeftAngle[0]) autoTurnLeft(middleLeftAngle[0]);
						else {
							autoStop();
							step = 3;
						}
					}
					else if (step == 3) {
						if (straightDistance < middleLeftDist[1]) {
							autoBackward(correctionAngle, middleLeftDist[1]);

							if (!intake->Get()) intake->Set(true);

							if (straightDistance > 15) pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);

						}
						else {
							autoStop();
							step = 4;
							pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);
						}
					}
					else if (step == 4) {
						if (fabs(gyro->GetAngle()) < middleLeftAngle[1]) autoTurnRight(middleLeftAngle[1]);
						else {
							autoStop();
							step = 5;
						}
					}
					else if (step == 5) {
						if (straightDistance < middleLeftDist[2]) autoBackward(correctionAngle, middleLeftDist[2]);
						else {
							autoStop();
							step = 6;
						}
					}
					else if (step == 6) {
						autoBackwardSlow(0.2);
						Wait(0.75);
						autoStop();
						autoShoot();
						Wait(0.75);
						autoStop();
						step = 7;
					}
					else if (step == 7) {
						if (straightDistance < middleLeftDist[3]) autoStraight(correctionAngle, middleLeftDist[3]);
						else {
							autoStop();
							step = 8;
						}
					}
					else if (step == 8) {
						if (fabs(gyro->GetAngle()) < middleLeftAngle[2]) autoTurnLeft(middleLeftAngle[2]);
						else {
							autoStop();
							step = 9;
						}
					}
					else if (step == 9) {
						if (straightDistance < middleLeftDist[4]) autoStraight(correctionAngle, middleLeftDist[4]);
						else {
							autoStop();
							step = 10;
						}
					}
					else if (step == 10) {

						if (!blockSensor2->Get()) {
							autoForwardSlow(0.2);
							autoIntake();
						}
						else {
							middleLeftDist[5] = straightDistance * 0.9;
							autoStop();
							step = 11;
						}

					}
					else if (step == 11) {
						if (straightDistance < middleLeftDist[5]) autoBackward(correctionAngle, middleLeftDist[5]);
						else {
							autoStop();
							step = 12;
						}
					}
					else if (step == 12) {
						if (fabs(gyro->GetAngle()) < middleLeftAngle[3]) autoTurnRight(middleLeftAngle[3]);
						else {
							autoStop();
							step = 13;
						}
					}
					else if (step == 13) {
						if (straightDistance < middleLeftDist[6]) autoBackward(correctionAngle, middleLeftDist[6]);
						else {
							autoStop();
							step = 14;
						}
					}
					else if (step == 14) {
						autoBackwardSlow(0.2);
						Wait(1);
						autoStop();
						autoShoot();
						Wait(2);
						autoStop();
						step = 15;
					}
				}
				else if (side == 2) {
					if (step == 1) {
						if (straightDistance < rightLeftDist[0]) {
							autoBackward(correctionAngle, rightLeftDist[0]);
							pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);
						}
						else {
							autoStop();
							step = 2;
						}
					}
				}
				else if (side == 0) {
					if (step == 1) {

						if (straightDistance < leftLeftDist[0]) {

							autoBackward(correctionAngle, leftLeftDist[0]);

							if (straightDistance > 10) {

								if (!intake->Get()) intake->Set(true);
								if (straightDistance < 70 && straightDistance > 30) pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
								else if (straightDistance > 70) pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);

							}
							else pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);

						}
						else {
							autoStop();
							step = 2;
						}

					}
					else if (step == 2) {
						if (fabs(gyro->GetAngle()) < leftLeftAngle[0]) autoTurnRight(leftLeftAngle[0]);
						else {
							autoStop();
							step = 3;
						}
					}
					else if (step == 3) {
						autoBackwardSlow(0.2);
						Wait(1.5);
						autoStop();
						autoShoot();
						Wait(0.5);
						autoStop();
						step = 4;
					}
				}

			}
			else if (gameData[0] == 'R') {
				if (side == 1) {
					if (step == 1) {

						if (straightDistance < middleRightDist[0]) {
							autoBackward(correctionAngle, middleRightDist[0]);
							pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);
						}
						else {
							autoStop();
							step = 2;
						}
					}
					else if (step == 2) {
						if (fabs(gyro->GetAngle()) < middleRightAngle[0]) autoTurnRight(middleRightAngle[0]);
						else {
							autoStop();
							step = 3;
						}
					}
					else if (step == 3) {
						if (straightDistance < middleRightDist[1]) {
							autoBackward(correctionAngle, middleRightDist[1]);

							if (!intake->Get()) intake->Set(true);

							if (straightDistance > 15) pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);

						}
						else {
							autoStop();
							step = 4;
							pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);
						}
					}
					else if (step == 4) {
						if (fabs(gyro->GetAngle()) < middleRightAngle[1]) autoTurnLeft(middleRightAngle[1]);
						else {
							autoStop();
							step = 5;
						}
					}
					else if (step == 5) {
						if (straightDistance < middleRightDist[2]) autoBackward(correctionAngle, middleRightDist[2]);
						else {
							autoStop();
							step = 6;
						}
					}
					else if (step == 6) {

						autoBackwardSlow(0.2);
						Wait(0.75);
						autoStop();
						autoShoot();
						Wait(0.75);
						autoStop();
						step = 7;

					}
					else if (step == 7) {
						if (straightDistance < middleRightDist[3]) autoStraight(correctionAngle, middleRightDist[3]);
						else {
							autoStop();
							step = 8;
						}
					}
					else if (step == 8) {
						if (fabs(gyro->GetAngle()) < middleRightAngle[2]) autoTurnRight(middleRightAngle[2]);
						else {
							autoStop();
							step = 9;
						}
					}
					else if (step == 9) {
						if (straightDistance < middleRightDist[4]) autoStraight(correctionAngle, middleRightDist[4]);
						else {
							autoStop();
							step = 10;
						}
					}
					else if (step == 10) {

						if (!blockSensor2->Get()) {
							autoForwardSlow(0.2);
							autoIntake();
						}
						else {
							middleRightDist[5] = straightDistance * 0.9;
							autoStop();
							step = 11;
						}

					}
					else if (step == 11) {
						if (straightDistance < middleRightDist[5]) autoBackward(correctionAngle, middleRightDist[5]);
						else {
							autoStop();
							step = 12;
						}
					}
					else if (step == 12) {
						if (fabs(gyro->GetAngle()) < middleRightAngle[3]) autoTurnLeft(middleRightAngle[3]);
						else {
							autoStop();
							step = 13;
						}
					}
					else if (step == 13) {
						if (straightDistance < middleRightDist[6]) autoBackward(correctionAngle, middleRightDist[6]);
							else {
								autoStop();
								step = 14;
							}
					}
					else if (step == 14) {
						autoBackwardSlow(0.2);
						Wait(1);
						autoStop();
						autoShoot();
						Wait(2);
						autoStop();
						step = 15;
					}
				}
				else if (side == 2) {
					if (step == 1) {

						if (straightDistance < rightRightDist[0]) {

							autoBackward(correctionAngle, rightRightDist[0]);

							if (straightDistance > 10) {

								if (!intake->Get()) intake->Set(true);
								if (straightDistance < 70 && straightDistance > 30) pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
								else if (straightDistance > 70) pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);

							}
							else pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);

						}
						else {
							autoStop();
							step = 2;
						}

					}
					else if (step == 2) {

						if (fabs(gyro->GetAngle()) < rightRightAngle[0]) autoTurnLeft(rightRightAngle[0]);
						else {
							autoStop();
							step = 3;
						}

					}
					else if (step == 3) {

						autoBackwardSlow(0.2);
						Wait(1.5);
						autoStop();
						autoShoot();
						Wait(0.5);
						autoStop();
						step = 4;

					}

				}
				else if (side == 0) {
					if (step == 1) {
						if (straightDistance < leftRightDist[0]) {
							autoBackward(correctionAngle, leftRightDist[0]);
							pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);
						}
						else {
							autoStop();
							step = 2;
						}
					}
				}
			}
		}
		lastSumAngle = sumAngle;

	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.

		//Set some stuff that should be set before teleop starts
		intake->Set(false);
		rampDeploy->Set(false);
		leftRamp->Set(false);
		rightRamp->Set(false);
		leftEnc->Reset();
		rightEnc->Reset();
		gyro->Reset();

	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();

		//Set yInput to the drivers joystick Y-axis and xInput to the Wheels X-axis
		//yInput is used for speed and xInput is used for turning
		//There isnt really a point to assigning these things to variables other than to make things neater
		yInput = JoyAccel->GetY();
		xInput = RaceWheel->GetX();

		//This stuff finds the difference in the bots angle when driving straight and uses it to determine how much power to cut from one side to straighten out
		float sumAngle = gyro->GetAngle();
		float derivAngle = sumAngle - lastSumAngle;
		float correctionAngle = (sumAngle * 0.1) + (derivAngle * .2);

		if (JoyAccel->GetRawButton(7)) {
			rampDeploy->Set(true);
		}

		if (JoyAccel->GetRawButton(11)) {
			leftRamp->Set(true);
		}

		//if (JoyAccel->GetRawButton(10)) {
			//rightRamp->Set(true);
		//}

		std::cout << "X: " << table->GetNumber("tx",0.0) << std::endl;

		//Print out the encoder readings so you can test if theyre working
		//std::cout << "right: " << rightEnc->GetDistance() << std::endl;
		//std::cout << "left: " << leftEnc->GetDistance() << std::endl;
		//std::cout << "gyro: " << gyro->GetAngle() << std::endl;

		//if (!switch1->Get() && !switch2->Get()) side = 0;
		//else if (switch1->Get() && !switch2->Get()) side = 1;
		//else if (!switch1->Get() && switch2->Get()) side = 2;
		//std::cout << "6: " << switch1->Get() << std::endl;
		//std::cout << "8: " << switch2->Get() << std::endl;
		//std::cout << side << std::endl;

		//std::cout << "X: " << table->GetNumber("tx",0.0) << std::endl;
		//std::cout << "Y: " << table->GetNumber("ty",0.0) << std::endl;

		//Button 5 on the wheel activates point turning
		if (RaceWheel->GetRawButton(5)) {
			rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xInput);
			rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xInput);
			leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xInput);
			leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, xInput);
			gyro->Reset();
		}

		//Button 11 on the wheel activates vision tracking
		else if (RaceWheel->GetRawButton(11)) {

			float threshold = 1;
			float slowDownAngle = 3;
			float slowDown = 0.3;

			if (table->GetNumber("tx",0.0) < slowDownAngle && table->GetNumber("tx",0.0) > -slowDownAngle) {
				slowDown = 0.2;
				std::cout << "Slow Down Angle Reached" << std::endl;
			}
			if (table->GetNumber("tx",0.0) > threshold){
				std::cout << "Crosshair Left" << std::endl;
				rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, slowDown);
				rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, slowDown);
				leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, slowDown);
				leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, slowDown);
				gyro->Reset();
			} else if (table->GetNumber("tx",0.0) < -threshold){
				std::cout << "Crosshair Right" << std::endl;
				rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -slowDown);
				rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -slowDown);
				leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -slowDown);
				leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -slowDown);
				gyro->Reset();
			} else {
				std::cout << "Centered" << std::endl;
				rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.3);
				rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.3);
				leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
				leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
				gyro->Reset();

			}

		}
		else {

			//Code for regular turning
			if ((xInput < -0.01 || xInput > 0.01) && (yInput > 0.06 || yInput < -0.06)) {
				rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, yInput + turnFact*(xInput));
				rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, yInput + turnFact*(xInput));
				leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -yInput + turnFact*(xInput));
				leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -yInput + turnFact*(xInput));
				gyro->Reset();
			}
			//Code for driving straight
			else if ((yInput > 0.06 || yInput < -0.06)) {
				rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, yInput - correctionAngle);
				rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, yInput - correctionAngle);
				leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -yInput - correctionAngle);
				leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -yInput - correctionAngle);

			}
			else {
				//Dont spin any drive train motors if the driver is not doing anything
				rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);

			}
		}

		//Button 3 intakes
		if (JoyAccel2->GetRawButton(3)) {
			frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
			frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
			popUp->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
			//Only spin the arm intakes if the pivot is down for them
			if (!pivotBack) {
				intakeLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1.0);
				intakeRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1.0);
			}
			else {
				intakeLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				intakeRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			}

		}
		//Button 5 shoots
		else if (JoyAccel2->GetRawButton(5)) {
			leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.40);
			leftBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.40);
			rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.40);
			rightBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.40);
			frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
			frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
		}
		//Button 1 spits the cube out through the front
		else if (JoyAccel2->GetRawButton(1)) {
			frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.75);
			frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.75);
			leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
			rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
			popUp->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.6);
			//Only spin the arm intakes if the pivot is down for them
			if (!pivotBack) {
				intakeLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
				intakeRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.7);
			}
			else {
				intakeLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				intakeRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			}
			//Only spin the back conveyor motors if the cube is stuck back there
			if (!blockSensor1->Get() && !blockSensor2->Get()) {
				leftBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
				rightBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
			}
		}
		else {
			//Dont spin these motors if none of those buttons are being pressed
			leftBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			rightBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			popUp->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			intakeLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			intakeRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);

			//Code for the sensors on the pivot that keep the block in place while driving
			if (blockSensor1->Get()) {
				frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.3);
				frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.3);
				rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
				leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
			}
			else if (!blockSensor1->Get()) {

				if (!blockSensor2->Get()) {
					frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
					frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.3);
					rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				}
				else {
					frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
					leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				}
			}
		}

		//Pivot control with the left joystick on the controller
		if (JoyAccel2->GetY() > 0.1 || JoyAccel2->GetY() < -0.1) {

			if (JoyAccel2->GetY() > 0) {
				pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyAccel2->GetY() / 2);
				pivotBack = true;
			}
			//Make sure the pivot cannot be moved down onto the arms when the arms are in
			if (JoyAccel2->GetY() < 0 && intake->Get()) {
				pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, JoyAccel2->GetY() / 2);
				pivotBack = false;
			}

		}
		else {
			//Keep the pivot in place when the operator is not actively moving it
			if (!pivotBack) pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.1);
			else if (pivotBack) pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.1);
		}

		//Declare the emergency button as button 7
		if (JoyAccel2->GetRawButton(7) && !pivotBack) {
			EmergencyIntake = true;
		}

		//Do the emergency action when button 7 is pressed
		if (EmergencyIntake) {
			pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.75);
			pivotBack = true;
			if (!countingTime) {
				Emergency = time(0);
				countingTime = true;
			}
			else if (countingTime) {
				if (difftime(time(0), Emergency) > 0.95) {
					intake->Set(false);
					EmergencyIntake = false;
					countingTime = false;
					pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
				}
			}
		}

		//Toggle the intake arms with button 8 on the controller
		if (JoyAccel2->GetRawButton(8)) {

			if (!intakeWasPressed) {

				if (!intake->Get()) intake->Set(true);
				else if (intake->Get()) intake->Set(false);

				intakeWasPressed = true;

			}

		}
		else intakeWasPressed = false;

		//This is part of straightening out the bot when driving straight
		lastSumAngle = sumAngle;
	}

	void TestPeriodic() override {
		std::cout << pivot->GetSelectedSensorPosition(0) << std::endl;
	}

	void autoStraight(float angle, float distance) {
		if (power < 0.5) power = pow(straightDistance / distance,4);
		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -(1 - power) - angle);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -(1 - power) - angle);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, (1 - power) - angle);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, (1 - power) - angle);

	}

	void autoBackward(float angle, float distance) {
		if (power < 0.5) power = pow(straightDistance / distance,4);
		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, (1 - power) - angle);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, (1 - power) - angle);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -(1 - power) - angle);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -(1 - power) - angle);

	}

	void autoBackwardSlow(float power) {

		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -power);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -power);

	}

	void autoForwardSlow(float power) {

		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -power);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -power);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, power);

	}

	void autoStop() {
		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,0);
		leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		leftBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		rightBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		popUp->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		pivot->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		intakeLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		intakeRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
		power = 0;
		anglePower = 1;
		gyro->Reset();
		leftEnc->Reset();
		rightEnc->Reset();
	}

	void autoTurnRight(float angle) {
		if (anglePower > 0.25) anglePower =  0.8 - (fabs(gyro->GetAngle()) / angle)*0.8;
		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, anglePower);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, anglePower);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, anglePower);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, anglePower);
	}

	void autoTurnLeft(float angle) {
		if (anglePower > 0.25) anglePower =  0.8 - (fabs(gyro->GetAngle()) / angle)*0.8;
		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -anglePower);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -anglePower);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -anglePower);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -anglePower);
	}

	void telTurnRight(float angle) {
		if (anglePower > 0.25) anglePower =  0.4 - (fabs(gyro->GetAngle()) / angle)*0.4;
		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, anglePower);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, anglePower);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, anglePower);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, anglePower);
	}

	void telTurnLeft(float angle) {
		if (anglePower > 0.25) anglePower =  0.4 - (fabs(gyro->GetAngle()) / angle)*0.4;
		rightFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -anglePower);
		rightBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -anglePower);
		leftFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -anglePower);
		leftBack->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -anglePower);
	}

	void autoShoot() {
		leftMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.4);
		leftBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.4);
		rightMidConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.4);
		rightBackConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.4);
		frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
		frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
	}

	void autoIntake() {
		frontRightConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
		frontLeftConveyor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
		popUp->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
		intakeLeft->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
		intakeRight->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
	}

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
};

START_ROBOT_CLASS(Robot)
