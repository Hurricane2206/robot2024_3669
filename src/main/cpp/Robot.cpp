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
	defineAutoStateFunctions();
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	swerve.SetPosition(autoPose[0].pos);
	autoState = autoPose[0].startingState;
	lastAutoState = autoPose[0].startingState;
}
void Robot::AutonomousPeriodic() {
	ll.update();
	if (autoState != AutoState::AINTAKING) {
		intakeShooter.SetAngle(ll.pitch);
	}
	intakeShooter.RunAnglePID();
	swerve.RunPID(ll.tx);

	AutoPeriodic[autoState]();
	if (autoState != lastAutoState) {
		AutoInit[autoState]();
	}
	frc::SmartDashboard::PutNumber("angle", intakeShooter.GetAngle());
	frc::SmartDashboard::PutNumber("robot state", teleopState);
	frc::SmartDashboard::PutBoolean("note detected", intakeShooter.eye2.Get());
	frc::SmartDashboard::PutNumber("posx", swerve.pos.real());
	frc::SmartDashboard::PutNumber("posy", swerve.pos.imag());
	frc::SmartDashboard::PutNumber("posErrorx", swerve.posError.real());
	frc::SmartDashboard::PutNumber("posErrory", swerve.posError.imag());
	frc::SmartDashboard::PutNumber("posChangex", swerve.posChange.real());
	frc::SmartDashboard::PutNumber("posChangy", swerve.posChange.imag());
	frc::SmartDashboard::PutNumber("mod0Changex", swerve.GetModulePosChange(0).real());
	frc::SmartDashboard::PutNumber("mod0Changey", swerve.GetModulePosChange(0).imag());
	frc::SmartDashboard::PutNumber("mod1Changex", swerve.GetModulePosChange(1).real());
	frc::SmartDashboard::PutNumber("mod1Changey", swerve.GetModulePosChange(1).imag());
	frc::SmartDashboard::PutNumber("mod2Changex", swerve.GetModulePosChange(2).real());
	frc::SmartDashboard::PutNumber("mod2Changey", swerve.GetModulePosChange(2).imag());
	frc::SmartDashboard::PutNumber("mod3Changex", swerve.GetModulePosChange(3).real());
	frc::SmartDashboard::PutNumber("mod3Changey", swerve.GetModulePosChange(3).imag());
	frc::SmartDashboard::PutNumber("motor0Change", swerve.GetMotorPosChange(0));
	frc::SmartDashboard::PutNumber("motor1Change", swerve.GetMotorPosChange(1));
	frc::SmartDashboard::PutNumber("motor2Change", swerve.GetMotorPosChange(2));
	frc::SmartDashboard::PutNumber("motor3Change", swerve.GetMotorPosChange(3));
	frc::SmartDashboard::PutNumber("motor0Pos", swerve.GetMotorPos(0));
	frc::SmartDashboard::PutNumber("motor1Pos", swerve.GetMotorPos(1));
	frc::SmartDashboard::PutNumber("motor2Pos", swerve.GetMotorPos(2));
	frc::SmartDashboard::PutNumber("motor3Pos", swerve.GetMotorPos(3));

	lastAutoState = autoState;
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic(){
	float tROffset = 0;
	lastTeleopState = teleopState;
	ll.update();
	frc::SmartDashboard::PutNumber("LimelightTY: ", ll.ty);
	tROffset = -ll.tx / 35.0;
	TelPeriodic[teleopState]();
	if (teleopState != lastTeleopState) {TelInit[teleopState]();}
	// this switch case runs for each state
	// switch (teleopState) {
	// 	case TeleopState::AIMING:
	// 		if (key_pad.GetRawButton(12) && intakeShooter.GetNotePresent()){
	// 			timer.Restart();
	// 			teleopState = TeleopState::RAMPING;
	// 		}
	// 		if (!key_pad.GetRawButton(11)) {
	// 			teleopState = TeleopState::DEFAULT;
	// 		}
	// 		intakeShooter.SetAngle(pitch);
	// 		break;
	// 	case TeleopState::RAMPING:
	// 		if (timer.HasElapsed(1.1_s)){
	// 			teleopState = TeleopState::SHOOTING;
	// 		}
	// 		break;
	// 	case TeleopState::SHOOTING:
	// 		if (timer.HasElapsed(1.3_s)){
	// 			teleopState = TeleopState::DEFAULT;
	// 		}
	// 		break;
	// 	// Amp procedure
	// 	case TeleopState::AMPTRANSFER:
	// 		if (!intakeShooter.GetNotePresent()) {
	// 			teleopState = TeleopState::AMPOS;
	// 		}
	// 		break;
	// 	case TeleopState::AMPOS:
	// 		if (key_pad.GetRawButtonPressed(2)) {
	// 			teleopState = TeleopState::AMPSCORE;
	// 		}
	// 		break;
	// 	case TeleopState::AMPSCORE:
	// 		if (timer.HasElapsed(0.5_s)) {
	// 			teleopState = TeleopState::ARMDEFAULT;
	// 		}
	// 		break;
	// 	case TeleopState::DEFENDING:
	// 		if (key_pad.GetRawButtonPressed(1)) {
	// 			teleopState = TeleopState::ARMDEFAULT;
	// 		}
	// 		break;
	// 	case TeleopState::ARMDEFAULT:
	// 		if (timer.HasElapsed(0.7_s)) {
	// 			teleopState = TeleopState::DEFAULT;
	// 		}
	// 		break;
	// 	// climb procedure
	// 	case TeleopState::TRAPTRANSFER:
	// 		if (!intakeShooter.GetNotePresent()){
	// 			teleopState = TeleopState::INTAKECLEAR;
	// 		}
	// 		break;
	// 	case TeleopState::INTAKECLEAR:
	// 		if (intakeShooter.GetAngle() > 50) {
	// 			teleopState = TeleopState::TRAPCLIMBUP;
	// 		}
	// 		break;
	// 	case TeleopState::TRAPCLIMBUP:
	// 		if (key_pad.GetRawButtonPressed(3)){
	// 			teleopState = TeleopState::TRAPCLIMBDOWN;
	// 		}
	// 		break;
	// 	case TeleopState::TRAPCLIMBDOWN:
	// 		if (climb.GetHeightReached(0)) {
	// 			teleopState = TeleopState::WAITFORCLIMB;
	// 		}
	// 		break;
	// 	case TeleopState::WAITFORCLIMB:
	// 		if (timer.HasElapsed(1_s)) {
	// 			teleopState = TeleopState::TRAPSCOREREADY;
	// 		}
	// 		break;
	// 	case TeleopState::TRAPSCOREREADY:
	// 		if (timer.HasElapsed(0.4_s)) {
	// 			teleopState = TeleopState::WAITINGTOSCORE;
	// 		}
	// 		break;
	// 	case TeleopState::WAITINGTOSCORE:
	// 		if (timer.HasElapsed(1.5_s)) {
	// 			teleopState = TeleopState::TRAPSCORE;
	// 		}
	// 		break;
	// 	case TeleopState::TRAPSCORE:
	// 		if (timer.HasElapsed(1.5_s)) {
	// 			arm.SetRollerSpeed(0);
	// 		}
	// 		break;
	// 	case TeleopState::INTAKING:
	// 		if (key_pad.GetRawButtonPressed(10)) {
	// 			teleopState = TeleopState::DEFAULT;
	// 		} else if (intakeShooter.eye0.Get()) {
	// 			teleopState = TeleopState::NOTEALIGN1;
	// 		}
	// 		break;
	// 	case TeleopState::NOTEALIGN1:
	// 		if (intakeShooter.eye2.Get()) {
	// 			intakeShooter.SetIntakeSpeed(-50);
	// 			intakeShooter.SetShooter(-10);
	// 		} else {
	// 			teleopState = TeleopState::NOTEALIGN2;
	// 		}
	// 		break;
	// 	case TeleopState::NOTEALIGN2:
	// 		if (!intakeShooter.eye2.Get()) {
	// 			intakeShooter.SetIntakeSpeed(50);
	// 			intakeShooter.SetShooter(0);
	// 		} else {
	// 			teleopState = TeleopState::NOTEALIGN3;
	// 		}
	// 		break;
	// 	case TeleopState::NOTEALIGN3:
	// 		if (intakeShooter.GetNotePresent()) {
	// 			teleopState = TeleopState::DEFAULT;
	// 		}
	// 		break;
	// 	case TeleopState::DEFAULT:
	// 		if (key_pad.GetRawButtonPressed(10) && !intakeShooter.GetNotePresent()) {
	// 			teleopState = TeleopState::INTAKING;
	// 		}
	// 		if (key_pad.GetRawButton(11)) {
	// 			teleopState = TeleopState::AIMING;
	// 		}
	// 		if (key_pad.GetRawButtonPressed(1)) {
	// 			teleopState = TeleopState::DEFENDING;
	// 		}
	// 		if (key_pad.GetRawButtonPressed(2)) {
	// 			teleopState = TeleopState::AMPTRANSFER;
	// 		}
	// 		if (key_pad.GetRawButtonPressed(3)) {
	// 			teleopState = TeleopState::TRAPTRANSFER;
	// 		}
	// 		break;
	// }

	// this switch case only runs when the robot state changes
	// if (teleopState != lastTeleopState) {
	// 	switch (teleopState) {
	// 		// case AIMING:
	// 		// 	intakeShooter.SetP(0.01);
	// 		// 	intakeShooter.SetOutputRange(-0.6, 0.7);
	// 		// 	break;
	// 		case TeleopState::RAMPING:
	// 			intakeShooter.SetShooter(60);
	// 			break;
	// 		case TeleopState::SHOOTING:
	// 			intakeShooter.SetIntake(100);
	// 			break;
	// 		case TeleopState::INTAKING:
	// 			intakeShooter.SetAngle(94);
	// 			intakeShooter.SetIntake(80);
	// 			break;
	// 		case TeleopState::NOTEALIGN1:
	// 			intakeShooter.SetAngle(15);
	// 			intakeShooter.SetIntakeSpeed(50);
	// 			break;
	// 		// amp procedure:
	// 		case TeleopState::AMPTRANSFER:
	// 			arm.SetAngle(20);
	// 			arm.SetHeight(0);
	// 			arm.SetRollerSpeed(130);
	// 			intakeShooter.SetAngle(15);
	// 			intakeShooter.SetIntakeSpeed(100);
	// 			intakeShooter.SetShooter(10);
	// 			break;
	// 		case TeleopState::AMPOS:
	// 			intakeShooter.SetAngle(40);
	// 			intakeShooter.SetShooter(0);
	// 			intakeShooter.SetIntake(0);
	// 			arm.SetRollerSpeed(0);
	// 			arm.SetHeight(15);
	// 			arm.SetAngle(225);
	// 			break;
	// 		case TeleopState::AMPSCORE:
	// 			timer.Restart();
	// 			arm.SetRollerSpeed(-100);
	// 			break;
	// 		case TeleopState::DEFENDING:
	// 			arm.SetHeight(20);
	// 			arm.SetAngle(180);
	// 			break;
	// 		case TeleopState::ARMDEFAULT:
	// 			timer.Restart();
	// 			intakeShooter.SetAngle(40);
	// 			arm.SetHeight(0);
	// 			arm.SetAngle(0);
	// 			break;
	// 		// trap procedure:
	// 		case TeleopState::TRAPTRANSFER:
	// 			arm.SetAngle(20);
	// 			arm.SetHeight(0);
	// 			arm.SetRollerSpeed(130);
	// 			intakeShooter.SetAngle(15);
	// 			intakeShooter.SetIntakeSpeed(100);
	// 			intakeShooter.SetShooter(10);
	// 			break;
	// 		case TeleopState::INTAKECLEAR:
	// 			arm.SetAngle(90);
	// 			arm.SetHeight(0);
	// 			arm.SetRollerSpeed(0);
	// 			intakeShooter.SetAngle(60);
	// 			intakeShooter.SetIntakeSpeed(0);
	// 			intakeShooter.SetShooter(0);
	// 			break;
	// 		case TeleopState::TRAPCLIMBUP:
	// 			intakeShooter.SetAngle(60);
	// 			arm.SetAngle(90);
	// 			arm.SetHeight(0);
	// 			arm.SetRollerSpeed(0);
	// 			intakeShooter.SetIntakeSpeed(0);
	// 			intakeShooter.SetShooter(0);
	// 			climb.SetHeight(20);
	// 			break;
	// 		case TeleopState::TRAPCLIMBDOWN:
	// 			intakeShooter.SetAngle(60);
	// 			arm.SetAngle(90);
	// 			arm.SetHeight(20.5);
	// 			arm.SetRollerSpeed(0);
	// 			intakeShooter.SetIntakeSpeed(0);
	// 			intakeShooter.SetShooter(0);
	// 			climb.SetHeight(0);
	// 			break;
	// 		case TeleopState::WAITFORCLIMB:
	// 			timer.Restart();
	// 			break;
	// 		case TeleopState::TRAPSCOREREADY:
	// 			timer.Restart();
	// 			intakeShooter.SetAngle(15);
	// 			arm.SetAngle(235);
	// 			arm.SetHeight(20.5);
	// 			arm.SetRollerSpeed(50);
	// 			intakeShooter.SetIntakeSpeed(0);
	// 			intakeShooter.SetShooter(0);
	// 			climb.SetHeight(0);
	// 			break;
	// 		case TeleopState::WAITINGTOSCORE:
	// 			timer.Restart();
	// 			arm.SetRollerSpeed(0);
	// 			break;
	// 		case TeleopState::TRAPSCORE:
	// 			timer.Restart();
	// 			intakeShooter.SetAngle(15);
	// 			arm.SetAngle(235);
	// 			arm.SetHeight(20.5);
	// 			arm.SetRollerSpeed(-100);
	// 			intakeShooter.SetIntakeSpeed(0);
	// 			intakeShooter.SetShooter(0);
	// 			climb.SetHeight(0);
	// 			break;
	// 		case TeleopState::DEFAULT:
	// 			intakeShooter.SetAngle(15);
	// 			intakeShooter.SetIntakeSpeed(0);
	// 			intakeShooter.SetShooter(0);
	// 			arm.SetAngle(0);
	// 			arm.SetHeight(0);
	// 			arm.SetRollerSpeed(0);
	// 			break;
	// 	}
	// }
	float x = -controller.GetLeftY();
	float y = -controller.GetLeftX();
	float tR = -controller.GetRightX() + tROffset;
	float rt = (controller.GetRightTriggerAxis()*0.5)+0.5;
	complex<float> velocity = complex<float>(x*rt, y*rt);
	float turnRate = tR*0.3;
	swerve.set(velocity, turnRate);
	intakeShooter.RunAnglePID();
	if (controller.GetAButtonPressed()) {
		swerve.resetPos();
	}

	frc::SmartDashboard::PutNumber("angle", intakeShooter.GetAngle());
	frc::SmartDashboard::PutNumber("robot state", teleopState);
	frc::SmartDashboard::PutBoolean("note detected", intakeShooter.eye2.Get());
	frc::SmartDashboard::PutNumber("posx", swerve.pos.real());
	frc::SmartDashboard::PutNumber("posy", swerve.pos.imag());
	frc::SmartDashboard::PutNumber("posErrorx", swerve.posError.real());
	frc::SmartDashboard::PutNumber("posErrory", swerve.posError.imag());
	frc::SmartDashboard::PutNumber("posChangex", swerve.posChange.real());
	frc::SmartDashboard::PutNumber("posChangy", swerve.posChange.imag());
	frc::SmartDashboard::PutNumber("mod0Changex", swerve.GetModulePosChange(0).real());
	frc::SmartDashboard::PutNumber("mod0Changey", swerve.GetModulePosChange(0).imag());
	frc::SmartDashboard::PutNumber("mod1Changex", swerve.GetModulePosChange(1).real());
	frc::SmartDashboard::PutNumber("mod1Changey", swerve.GetModulePosChange(1).imag());
	frc::SmartDashboard::PutNumber("mod2Changex", swerve.GetModulePosChange(2).real());
	frc::SmartDashboard::PutNumber("mod2Changey", swerve.GetModulePosChange(2).imag());
	frc::SmartDashboard::PutNumber("mod3Changex", swerve.GetModulePosChange(3).real());
	frc::SmartDashboard::PutNumber("mod3Changey", swerve.GetModulePosChange(3).imag());
	frc::SmartDashboard::PutNumber("motor0Change", swerve.GetMotorPosChange(0));
	frc::SmartDashboard::PutNumber("motor1Change", swerve.GetMotorPosChange(1));
	frc::SmartDashboard::PutNumber("motor2Change", swerve.GetMotorPosChange(2));
	frc::SmartDashboard::PutNumber("motor3Change", swerve.GetMotorPosChange(3));
	frc::SmartDashboard::PutNumber("motor0Pos", swerve.GetMotorPos(0));
	frc::SmartDashboard::PutNumber("motor1Pos", swerve.GetMotorPos(1));
	frc::SmartDashboard::PutNumber("motor2Pos", swerve.GetMotorPos(2));
	frc::SmartDashboard::PutNumber("motor3Pos", swerve.GetMotorPos(3));
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}


   //\\     //    // ////////  //////
  //  \\    //    //    //    //    //
 /////*\\   //    //    //    //    //
//      \\   //////     //     //////

void defineAutoStateFunctions() {
	AutoInit[ADRIVING] = []() {
		intakeShooter.SetAngle(15);
		intakeShooter.SetIntake(0);
		intakeShooter.SetShooter(0);
		arm.SetAngle(0);
		arm.SetHeight(0);
		arm.SetRollerSpeed(0);
	};
	AutoPeriodic[ADRIVING] = []() {
		if (swerve.GetPositionReached() && x < size(autoPose)-1) {
			x++;
			swerve.SetPosition(autoPose[x].pos);
			autoState = autoPose[x].startingState;
		}
	};

	AutoInit[AINTAKING] = []() {
		intakeShooter.SetAngle(91);
		intakeShooter.SetIntake(70);
	};
	AutoPeriodic[AINTAKING] = []() {
		if (intakeShooter.eye0.Get()) {
			autoState = AutoState::ANOTEALIGN1;
		}
	};

	AutoInit[AAIMING] = []() {};
	AutoPeriodic[AAIMING] = []() {
		if (swerve.GetPositionReached() && intakeShooter.GetAngleReached(3) && abs(ll.tx) < 5 && ll.targetValid) {
			autoState = AutoState::ARAMPING;
		}
	};

	AutoInit[ARAMPING] = []() {
		timer.Restart();
		intakeShooter.SetShooter(60);
	};
	AutoPeriodic[ARAMPING] = []() {
		if (timer.HasElapsed(1_s)){
			autoState = AutoState::ASHOOTING;
		}
	};

	AutoInit[ASHOOTING] = []() {
		intakeShooter.SetIntake(100);
	};

	AutoPeriodic[ASHOOTING] = []() {
		if (timer.HasElapsed(1.3_s)){
			autoState = AutoState::ADRIVING;
		}
	};

	AutoInit[AutoState::ANOTEALIGN1] = []() {};
	AutoPeriodic[AutoState::ANOTEALIGN1] = []() {
		if (intakeShooter.eye2.Get()) {
			intakeShooter.SetIntakeSpeed(-50);
			intakeShooter.SetShooter(-10);
		} else {
			autoState = AutoState::ANOTEALIGN2;
		}
	};

	AutoInit[AutoState::ANOTEALIGN2] = []() {};
	AutoPeriodic[AutoState::ANOTEALIGN2] = []() {
		if (!intakeShooter.eye2.Get()) {
			intakeShooter.SetIntakeSpeed(50);
			intakeShooter.SetShooter(0);
		} else {
			autoState = AutoState::ANOTEALIGN3;
		}
	};

	AutoInit[ANOTEALIGN3] = []() {};
	AutoPeriodic[ANOTEALIGN3] = []() {
		if (intakeShooter.GetNotePresent()) {
			autoState = AutoState::ADRIVING;
		}
	};
}

////////// //////// //       ////////  ///////  //////
    //     //       //       //       //     // //   //
    //     /////    //       /////    //     // //////
    //     //       //       //       //     // //
    //     //////// //////// ////////  ///////  //

void defineTeleopStateFunctions() {
	TelInit[DEFAULT] = []() {
		intakeShooter.SetAngle(15);
		intakeShooter.SetIntakeSpeed(0);
		intakeShooter.SetShooter(0);
		arm.SetAngle(0);
		arm.SetHeight(0);
		arm.SetRollerSpeed(0);
	};
	TelPeriodic[DEFAULT] = []() {
		if (key_pad.GetRawButtonPressed(10) && !intakeShooter.GetNotePresent()) {
			teleopState = TeleopState::INTAKING;
		}
		if (key_pad.GetRawButton(11)) {
			teleopState = TeleopState::AIMING;
		}
		if (key_pad.GetRawButtonPressed(1)) {
			teleopState = TeleopState::DEFENDING;
		}
		if (key_pad.GetRawButtonPressed(2)) {
			teleopState = TeleopState::AMPTRANSFER;
		}
		if (key_pad.GetRawButtonPressed(3)) {
			teleopState = TeleopState::TRAPTRANSFER;
		}
	};

	TelInit[AIMING] = []() {};
	TelPeriodic[AIMING] = []() {
		if (key_pad.GetRawButton(12) && intakeShooter.GetNotePresent()){
			timer.Restart();
			teleopState = TeleopState::RAMPING;
		}
		if (!key_pad.GetRawButton(11)) {
			teleopState = TeleopState::DEFAULT;
		}
		intakeShooter.SetAngle(ll.pitch);
	};

	TelInit[RAMPING] = []() {
		intakeShooter.SetShooter(60);
	};
	TelPeriodic[RAMPING] = []() {
		if (timer.HasElapsed(1.1_s)){
			teleopState = TeleopState::SHOOTING;
		}
	};

	TelInit[SHOOTING] = []() {
		intakeShooter.SetIntake(100);
	};
	TelPeriodic[SHOOTING] = []() {
		if (timer.HasElapsed(1.3_s)){
			teleopState = TeleopState::DEFAULT;
		}
	};

	TelInit[AMPTRANSFER] = []() {
		arm.SetAngle(20);
		arm.SetHeight(0);
		arm.SetRollerSpeed(130);
		intakeShooter.SetAngle(15);
		intakeShooter.SetIntakeSpeed(100);
		intakeShooter.SetShooter(10);
	};
	TelPeriodic[AMPTRANSFER] = []() {
		if (!intakeShooter.GetNotePresent()){
			teleopState = TeleopState::INTAKECLEAR;
		}
	};

	TelInit[AMPOS] = []() {
		intakeShooter.SetAngle(40);
		intakeShooter.SetShooter(0);
		intakeShooter.SetIntake(0);
		arm.SetRollerSpeed(0);
		arm.SetHeight(15);
		arm.SetAngle(225);
	};
	TelPeriodic[AMPOS] = []() {
		if (key_pad.GetRawButtonPressed(2)) {
			teleopState = TeleopState::AMPSCORE;
		}
	};

	TelInit[AMPSCORE] = []() {
		timer.Restart();
		arm.SetRollerSpeed(-100);
	};
	TelPeriodic[AMPSCORE] = []() {
		if (timer.HasElapsed(0.5_s)) {
			teleopState = TeleopState::ARMDEFAULT;
		}
	};

	TelInit[DEFENDING] = []() {
		arm.SetHeight(20);
		arm.SetAngle(180);
	};
	TelPeriodic[DEFENDING] = []() {
		if (key_pad.GetRawButtonPressed(1)) {
			teleopState = TeleopState::ARMDEFAULT;
		}
	};

	TelInit[ARMDEFAULT] = []() {
		timer.Restart();
		intakeShooter.SetAngle(40);
		arm.SetHeight(0);
		arm.SetAngle(0);
	};
	TelPeriodic[ARMDEFAULT] = []() {
		if (timer.HasElapsed(0.7_s)) {
			teleopState = TeleopState::DEFAULT;
		}
	};
	
	TelInit[TRAPTRANSFER] = []() {
		arm.SetAngle(20);
		arm.SetHeight(0);
		arm.SetRollerSpeed(130);
		intakeShooter.SetAngle(15);
		intakeShooter.SetIntakeSpeed(100);
		intakeShooter.SetShooter(10);
	};
	TelPeriodic[TRAPTRANSFER] = []() {
		if (!intakeShooter.GetNotePresent()){
			teleopState = TeleopState::INTAKECLEAR;
		}
	};

	TelInit[INTAKECLEAR] = []() {
		arm.SetAngle(90);
		arm.SetHeight(0);
		arm.SetRollerSpeed(0);
		intakeShooter.SetAngle(60);
		intakeShooter.SetIntakeSpeed(0);
		intakeShooter.SetShooter(0);
	};
	TelPeriodic[INTAKECLEAR] = []() {
		if (intakeShooter.GetAngle() > 50) {
			teleopState = TeleopState::TRAPCLIMBUP;
		}
	};

	TelInit[TRAPCLIMBUP] = []() {
		intakeShooter.SetAngle(60);
		arm.SetAngle(90);
		arm.SetHeight(0);
		arm.SetRollerSpeed(0);
		intakeShooter.SetIntakeSpeed(0);
		intakeShooter.SetShooter(0);
		climb.SetHeight(20);
	};
	TelPeriodic[TRAPCLIMBUP] = []() {
		if (key_pad.GetRawButtonPressed(3)){
			teleopState = TeleopState::TRAPCLIMBDOWN;
		}
	};

	TelInit[TRAPCLIMBDOWN] = []() {
		intakeShooter.SetAngle(60);
		arm.SetAngle(90);
		arm.SetHeight(20.5);
		arm.SetRollerSpeed(0);
		intakeShooter.SetIntakeSpeed(0);
		intakeShooter.SetShooter(0);
		climb.SetHeight(0);
	};
	TelPeriodic[TRAPCLIMBDOWN] = []() {
		if (climb.GetHeightReached(0)) {
			teleopState = TeleopState::WAITFORCLIMB;
		}
	};

	TelInit[WAITFORCLIMB] = []() {
		timer.Restart();
	};
	TelPeriodic[WAITFORCLIMB] = []() {
		if (timer.HasElapsed(1_s)) {
			teleopState = TeleopState::TRAPSCOREREADY;
		}
	};

	TelInit[TRAPSCOREREADY] = []() {
		timer.Restart();
		intakeShooter.SetAngle(15);
		arm.SetAngle(235);
		arm.SetHeight(20.5);
		arm.SetRollerSpeed(50);
		intakeShooter.SetIntakeSpeed(0);
		intakeShooter.SetShooter(0);
		climb.SetHeight(0);
	};
	TelPeriodic[TRAPSCOREREADY] = []() {
		if (timer.HasElapsed(0.4_s)) {
			teleopState = TeleopState::WAITINGTOSCORE;
		}
	};
	
	TelInit[WAITINGTOSCORE] = []() {
		timer.Restart();
		arm.SetRollerSpeed(0);
	};
	TelPeriodic[WAITINGTOSCORE] = []() {
		if (timer.HasElapsed(1.5_s)) {
			teleopState = TeleopState::TRAPSCORE;
		}
	};
	
	TelInit[TRAPSCORE] = []() {
		timer.Restart();
		intakeShooter.SetAngle(15);
		arm.SetAngle(235);
		arm.SetHeight(20.5);
		arm.SetRollerSpeed(-100);
		intakeShooter.SetIntakeSpeed(0);
		intakeShooter.SetShooter(0);
		climb.SetHeight(0);
	};
	TelPeriodic[TRAPSCORE] = []() {
		if (timer.HasElapsed(1.5_s)) {
			arm.SetRollerSpeed(0);
		}
	};

	TelInit[INTAKING] = []() {
		intakeShooter.SetAngle(94);
		intakeShooter.SetIntake(80);
	};
	TelPeriodic[INTAKING] = []() {
		if (key_pad.GetRawButtonPressed(10)) {
			teleopState = TeleopState::DEFAULT;
		} else if (intakeShooter.eye0.Get()) {
			teleopState = TeleopState::NOTEALIGN1;
		}
	};

	TelInit[NOTEALIGN1] = []() {
		intakeShooter.SetAngle(15);
		intakeShooter.SetIntakeSpeed(50);
	};
	TelPeriodic[NOTEALIGN1] = []() {
		if (intakeShooter.eye2.Get()) {
			intakeShooter.SetIntakeSpeed(-50);
			intakeShooter.SetShooter(-10);
		} else {
			teleopState = TeleopState::NOTEALIGN2;
		}
	};

	TelInit[NOTEALIGN2] = []() {};
	TelPeriodic[NOTEALIGN2] = []() {
		if (!intakeShooter.eye2.Get()) {
			intakeShooter.SetIntakeSpeed(50);
			intakeShooter.SetShooter(0);
		} else {
			teleopState = TeleopState::NOTEALIGN3;
		}
	};

	TelInit[NOTEALIGN3] = []() {};
	TelPeriodic[NOTEALIGN3] = []() {
		if (intakeShooter.GetNotePresent()) {
			teleopState = TeleopState::DEFAULT;
		}
	};
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif