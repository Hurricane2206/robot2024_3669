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

void Robot::TeleopInit(){}
void Robot::TeleopPeriodic(){
	complex<float> velocity = complex<float>(-controller.GetLeftY(), -controller.GetLeftX());
	float turnRate = -controller.GetRightX()*0.3;
	swerve.set(velocity, turnRate);
	if (isShooting){
		intakeShooter.SetShooter(60);
		if (shootTimer.HasElapsed(1_s)){
			intakeShooter.SetIntake(100);
		}
		if (shootTimer.HasElapsed(1.3_s)){
			intakeShooter.SetShooter(0);
			intakeShooter.SetIntake(0);
			isShooting = false;
		}
	}
	else if (isTransfering){
		arm.SetAngle(20);
		arm.SetHeight(0);
		intakeShooter.SetAngle(5);
		if (intakeShooter.GetNotePresent()){
			intakeShooter.SetIntakeSpeed(20);
			intakeShooter.SetShooterSpeed(20);
			arm.SetRollerSpeed(20);
		}
		else {
			isTransfering = false;
			intakeShooter.SetIntake(0);
			intakeShooter.SetShooter(0);
			arm.SetRollerSpeed(0);
		}
	}
	else{
		if (key_pad.GetRawButton(11)){
			intakeShooter.SetAngle(30);
			// todo: aim swerve
			if (key_pad.GetRawButton(12)){
				shootTimer.Restart();
				isShooting = true;
			}
		}
		else if (key_pad.GetRawButtonPressed(2) && intakeShooter.GetNotePresent()){
			isTransfering = true;
		}
		else{
			if (isIntaking){
				intakeShooter.SetAngle(80);
				intakeShooter.SetIntake(70);
				if (intakeShooter.GetNotePresent() || key_pad.GetRawButtonPressed(10)){
					isIntaking = false;
				}
			}
			else {
				if (key_pad.GetRawButtonPressed(10) && !intakeShooter.GetNotePresent()){
					isIntaking = true;
				}
				intakeShooter.SetAngle(5);
				intakeShooter.SetIntake(0);
			}
		}
	}
	frc::SmartDashboard::PutNumber("angle", intakeShooter.GetAngle());
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
