// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "rev/CANSparkMax.h"

class IntakeShooter
{
public:
	bool SetAngle(float angle, float maxRPM, float tolerance) {
		anglePID.SetSmartMotionMaxVelocity(maxRPM);
		anglePID.SetReference(angle, rev::CANSparkMax::ControlType::kSmartMotion);
		return abs(angle - e_angle.GetPosition()) < tolerance;
	}
	// void SetIntakeSpeed(float inPerMin) {
	// 	intakePID.SetReference(inPerMin, rev::CANSparkMax::ControlType::kVelocity);
	// }
	// void SetShooterSpeed(float inPerMin) {
	// 	shooterPID1.SetReference(inPerMin, rev::CANSparkMax::ControlType::kVelocity);
	// 	shooterPID2.SetReference(inPerMin, rev::CANSparkMax::ControlType::kVelocity);
	// }
	// void SetShooter(float speed) {
	// 	m1_shooter.Set(speed);
	// 	m2_shooter.Set(speed);
	// }
	// bool GetNoteSensor() {
	// 	return true; // todo: get sensor value
	// }
	void Initialize() {
		// m_intake.RestoreFactoryDefaults();
		// m_intake.SetInverted(false);
		// intakePID.SetP(6e-5);
		// intakePID.SetI(1e-6);
		// intakePID.SetD(0);
		// intakePID.SetFF(0.000015);
		// e_intake.SetPositionConversionFactor(intakeIPR);
		// m_intake.BurnFlash();

		// m1_shooter.RestoreFactoryDefaults();
		// m1_shooter.SetInverted(false);
		// shooterPID1.SetP(6e-5);
		// shooterPID1.SetI(1e-6);
		// shooterPID1.SetD(0);
		// shooterPID1.SetFF(0.000015);
		// e1_shooter.SetPositionConversionFactor(shooterIPR);
		// m1_shooter.BurnFlash();

		// m2_shooter.RestoreFactoryDefaults();
		// m2_shooter.SetInverted(false);
		// shooterPID2.SetP(6e-5);
		// shooterPID2.SetI(1e-6);
		// shooterPID2.SetD(0);
		// shooterPID2.SetFF(0.000015);
		// e2_shooter.SetPositionConversionFactor(shooterIPR);
		// m2_shooter.BurnFlash();

		m_angle.RestoreFactoryDefaults();
		m_angle.SetInverted(false);
		anglePID.SetP(0.2);
		anglePID.SetI(0);
		anglePID.SetD(1);
		anglePID.SetFF(0);
		anglePID.SetSmartMotionMaxAccel(2000);
		e_angle.SetPositionConversionFactor(angleDPR);
		m_angle.BurnFlash();
	}
	
private:
	// rev::CANSparkMax m_intake{41, rev::CANSparkMax::MotorType::kBrushless};
	// rev::SparkPIDController intakePID = m_intake.GetPIDController();
	// rev::SparkRelativeEncoder e_intake = m_intake.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	// rev::CANSparkMax m1_shooter{42, rev::CANSparkMax::MotorType::kBrushless};
	// rev::SparkPIDController shooterPID1 = m1_shooter.GetPIDController();
	// rev::SparkRelativeEncoder e1_shooter = m1_shooter.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	// rev::CANSparkMax m2_shooter{43, rev::CANSparkMax::MotorType::kBrushless};
	// rev::SparkPIDController shooterPID2 = m2_shooter.GetPIDController();
	// rev::SparkRelativeEncoder e2_shooter = m2_shooter.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	rev::CANSparkMax m_angle{41, rev::CANSparkMax::MotorType::kBrushless};
	rev::SparkPIDController anglePID = m_angle.GetPIDController();
	rev::SparkRelativeEncoder e_angle = m_angle.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
	
	const float intakeGearboxReduction = 3;
	// inches per rotation of the intake motor
	const float intakeIPR = M_PI*2/intakeGearboxReduction;
	// inches per rotation of the shooter motors
	const float shooterIPR = M_PI*4;

	const float angleGearboxReduction = 3;
	// degrees per rotation of the angle motor
	const float angleDPR = 1.0/360/angleGearboxReduction;
};
