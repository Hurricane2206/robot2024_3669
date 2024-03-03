// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "math.h"

using namespace std;

void Robot::RobotInit()
{
	intakeShooter.Initialize();
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
	frc::SmartDashboard::PutBoolean("Sensor: ", dEye.Get());
	if (isShooting)
	{
		intakeShooter.SetShooter(0.25);
		if (shootTimer.HasElapsed(1_s))
		{
			intakeShooter.SetShooter(0);
			isShooting = false;
		}
	}
	else
	{
		complex<float> velocity = complex<float>(-controller.GetLeftY(), -controller.GetLeftX());
		float turnRate = -controller.GetRightX()*0.3;
		swerve.set(velocity, turnRate);
		if (controller.GetBButton())
		{
			intakeShooter.SetAngle(45);
			// todo: aim swerve
			if (controller.GetRightTriggerAxis() > 0.2)
			{
				shootTimer.Restart();
				isShooting = true;
			}
		}
		else
		{
			if (controller.GetAButton() && !intakeShooter.GetNotePresent())
			{
				intakeShooter.SetAngle(90);
				intakeShooter.SetIntakeSpeed(1000);
			}
			else
			{
				intakeShooter.SetAngle(0);
				intakeShooter.SetIntakeSpeed(0);
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
