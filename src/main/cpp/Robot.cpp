// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "math.h"

using namespace std;

void Robot::RobotInit()
{
	intakeShooter.init();
	swerve.init();
	arm.init();
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic()
{
	// if (swerve.setPos(autoPos[x].pos, autoPos[x].angle) && x < size(autoPos)-1){
	//   if (i == x) {
	//     posWaitTimer.Restart();
	//     i++;
	//   }
	//   if (posWaitTimer.HasElapsed(autoPos[x].time)) {
	//     x++;
	//   }
	// }
}

void Robot::TeleopInit() {
	intakeShooter.SetAngle(15);
}
void Robot::TeleopPeriodic(){
	lastRobotState = robotState;

	switch (robotState) {
		case AIMING:
			if (key_pad.GetRawButton(12) && intakeShooter.GetNotePresent()){
				timer.Restart();
				robotState = RAMPING;
			}
			if (!key_pad.GetRawButton(11)) {
				robotState = IDLE;
			}
			break;
		case RAMPING:
			// intakeShooter.SetShooter(60);
			if (timer.HasElapsed(1_s)){
				robotState = SHOOTING;
			}
			break;
		case SHOOTING:
			if (timer.HasElapsed(1.3_s)){
				robotState = IDLE;
			}
			break;
		case AMPTRANSFER:
			if (!intakeShooter.GetNotePresent()) {
				robotState = AMPOS;
			}
			break;
		case AMPOS:
			if (key_pad.GetRawButtonPressed(2)) {
				robotState = AMPSCORE;
			}
			break;
		case AMPSCORE:
			if (timer.HasElapsed(0.5_s)) {
				robotState = ARMDEFAULT;
			}
			break;
		case ARMDEFAULT:
			if (timer.HasElapsed(0.7_s)) {
				robotState = IDLE;
			}
			break;
		case INTAKING:
			if (intakeShooter.eye2.Get() || key_pad.GetRawButtonPressed(10)){
				robotState = IDLE;
			}
			break;
		case IDLE:
			if (key_pad.GetRawButtonPressed(10) && !intakeShooter.GetNotePresent()){
				robotState = INTAKING;
			}
			if (key_pad.GetRawButton(11)){
				robotState = AIMING;
			}
			if (key_pad.GetRawButtonPressed(2)) {
				robotState = AMPTRANSFER;
			}
			break;
	}
	if (robotState != lastRobotState) {
		switch (robotState) {
			case AIMING:
				intakeShooter.SetAngle(40);
				// todo: aim swerve
				break;
			case RAMPING:
				intakeShooter.SetShooter(60);
				break;
			case SHOOTING:
				intakeShooter.SetIntake(100);
				break;
			case AMPTRANSFER:
				arm.SetAngle(20);
				arm.SetHeight(0);
				arm.SetRollerSpeed(130);
				intakeShooter.SetAngle(15);
				intakeShooter.SetIntakeSpeed(100);
				intakeShooter.SetShooter(10);
				break;
			case INTAKING:
				intakeShooter.SetAngle(90);
				intakeShooter.SetIntake(70);
				break;
			case AMPOS:
				intakeShooter.SetAngle(40);
				intakeShooter.SetShooter(0);
				intakeShooter.SetIntake(0);
				arm.SetRollerSpeed(0);
				arm.SetHeight(15);
				arm.SetAngle(225);
				break;
			case AMPSCORE:
				timer.Restart();
				arm.SetRollerSpeed(-100);
				break;
			case ARMDEFAULT:
				timer.Restart();
				intakeShooter.SetAngle(40);
				arm.SetHeight(0);
				arm.SetAngle(0);
				break;
			case IDLE:
				intakeShooter.SetAngle(15);
				intakeShooter.SetIntake(0);
				intakeShooter.SetShooter(0);
				arm.SetAngle(0);
				arm.SetHeight(0);
				arm.SetRollerSpeed(0);
				break;
		}
	}
	// float x = -controller.GetLeftY();
	// float y = -controller.GetLeftX();
	// float tR = -controller.GetRightX();
	// x = (abs(x) > 0.05) ? x : 0;
	// y = (abs(y) > 0.05) ? y : 0;
	// tR = (abs(tR) > 0.05) ? tR : 0;
	// complex<float> velocity = complex<float>(x ,y);
	// float turnRate = tR*0.3;
	// swerve.set(velocity, turnRate);
	intakeShooter.RunAnglePID();
		
	frc::SmartDashboard::PutNumber("angle", intakeShooter.GetAngle());
	frc::SmartDashboard::PutNumber("robot state", robotState);
	frc::SmartDashboard::PutBoolean("note detected", intakeShooter.eye2.Get());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
