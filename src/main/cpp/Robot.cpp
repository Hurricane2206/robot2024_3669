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
	climb.init();
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

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic(){
	float tROffset = 0;
	lastRobotState = robotState;
	frc::SmartDashboard::PutNumber("Pitch: ", pitch);
	// this switch case runs for each state
	switch (robotState) {
		case AIMING:
			if (key_pad.GetRawButton(12) && intakeShooter.GetNotePresent()){
				timer.Restart();
				robotState = RAMPING;
			}
			if (!key_pad.GetRawButton(11)) {
				intakeShooter.SetP();
				intakeShooter.SetRange();
				robotState = IDLE;
			}
			tROffset = -ll.getSpeakerYaw() / 35.0;
			ty = ll.getSpeakerPitch();
			pitch = 0.0038*pow(ty, 2)+0.6508*ty+65.2899;
			intakeShooter.SetAngle(pitch);
			intakeShooter.SetP(0.01);
			intakeShooter.SetOutputRange(-0.8, 0.9);
			break;
		case RAMPING:
			if (timer.HasElapsed(1_s)){
				robotState = SHOOTING;
			}
			break;
		case SHOOTING:
			if (timer.HasElapsed(1.3_s)){
				robotState = IDLE;
			}
			break;
		// Amp procedure
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
		// climb procedure
		case TRAPTRANSFER:
			if (!intakeShooter.GetNotePresent()){
				robotState = INTAKECLEAR;
			}
			break;
		case INTAKECLEAR:
			if (intakeShooter.GetAngle() > 50) {
				robotState = TRAPCLIMBUP;
			}
			break;
		case TRAPCLIMBUP:
			if (key_pad.GetRawButtonPressed(3)){
				robotState = TRAPCLIMBDOWN;
			}
			break;
		case TRAPCLIMBDOWN:
			if (climb.GetHeightReached(0)) {
				robotState = WAITFORCLIMB;
			}
			break;
		case WAITFORCLIMB:
			if (timer.HasElapsed(1_s)) {
				robotState = TRAPSCOREREADY;
			}
			break;
		case TRAPSCOREREADY:
			if (timer.HasElapsed(0.4_s)) {
				robotState = WAITINGTOSCORE;
			}
			break;
		case WAITINGTOSCORE:
			if (timer.HasElapsed(1.5_s)) {
				robotState = TRAPSCORE;
			}
			break;
		case TRAPSCORE:
			if (timer.HasElapsed(1.5_s)) {
				arm.SetRollerSpeed(0);
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
			if (key_pad.GetRawButtonPressed(3)){
				robotState = TRAPTRANSFER;
			}
			break;
	}

	// this switch case only runs when the robot state changes
	if (robotState != lastRobotState) {
		switch (robotState) {
			case AIMING:
			
				break;
			case RAMPING:
				intakeShooter.SetShooter(55);
				break;
			case SHOOTING:
				intakeShooter.SetIntake(100);
				break;
			case INTAKING:
				intakeShooter.SetAngle(91);
				intakeShooter.SetIntake(70);
				break;
			// amp procedure:
			case AMPTRANSFER:
				arm.SetAngle(20);
				arm.SetHeight(0);
				arm.SetRollerSpeed(130);
				intakeShooter.SetAngle(15);
				intakeShooter.SetIntakeSpeed(100);
				intakeShooter.SetShooter(10);
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
			// trap procedure:
			case TRAPTRANSFER:
				arm.SetAngle(20);
				arm.SetHeight(0);
				arm.SetRollerSpeed(130);
				intakeShooter.SetAngle(15);
				intakeShooter.SetIntakeSpeed(100);
				intakeShooter.SetShooter(10);
				break;
			case INTAKECLEAR:
				arm.SetAngle(90);
				arm.SetHeight(0);
				arm.SetRollerSpeed(0);
				intakeShooter.SetAngle(60);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
			break;
			case TRAPCLIMBUP:
				intakeShooter.SetAngle(60);
				arm.SetAngle(90);
				arm.SetHeight(0);
				arm.SetRollerSpeed(0);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				climb.SetHeight(20);
				break;
			case TRAPCLIMBDOWN:
				intakeShooter.SetAngle(60);
				arm.SetAngle(90);
				arm.SetHeight(20.5);
				arm.SetRollerSpeed(0);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				climb.SetHeight(0);
				break;
			case WAITFORCLIMB:
				timer.Restart();
				break;
			case TRAPSCOREREADY:
				timer.Restart();
				intakeShooter.SetAngle(15);
				arm.SetAngle(235);
				arm.SetHeight(20.5);
				arm.SetRollerSpeed(50);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				climb.SetHeight(0);
				break;
			case WAITINGTOSCORE:
				timer.Restart();
				arm.SetRollerSpeed(0);
				break;
			case TRAPSCORE:
				timer.Restart();
				intakeShooter.SetAngle(15);
				arm.SetAngle(235);
				arm.SetHeight(20.5);
				arm.SetRollerSpeed(-100);
				intakeShooter.SetIntakeSpeed(0);
				intakeShooter.SetShooter(0);
				climb.SetHeight(0);
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
	float x = -controller.GetLeftY();
	float y = -controller.GetLeftX();
	float tR = -controller.GetRightX() + tROffset;
	float rt = (controller.GetRightTriggerAxis()*0.5)+0.5;
	complex<float> velocity = complex<float>(x*rt, y*rt);
	float turnRate = tR*0.3;
	swerve.set(velocity, turnRate);
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
