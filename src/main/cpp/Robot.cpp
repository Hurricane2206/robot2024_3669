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
	else{
	 	complex<float> velocity = complex<float>(-controller.GetLeftY(), -controller.GetLeftX());
		float turnRate = -controller.GetRightX()*0.3;
		swerve.set(velocity, turnRate);
		if (kPad.GetRawButton(11)){
			intakeShooter.SetAngle(30);
			// todo: aim swerve
			if (kPad.GetRawButtonPressed(12)){
				shootTimer.Restart();
				isShooting = true;
			}
		}
		else{
			if (kPad.GetRawButtonPressed(10)){
				isIntaking = true;
			}
			if (isIntaking && !intakeShooter.GetNotePresent()){
				intakeShooter.SetAngle(80);
				intakeShooter.SetIntake(70);
			}
			else{
				isIntaking = false;
				intakeShooter.SetAngle(5);
				intakeShooter.SetIntake(0);
			}
		}
	}
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
