// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <ctre/phoenix6/TalonFX.hpp>

class IntakeShooter {
public:
 	bool SetAngle(float angle, float tolerance = 4) {
 		anglePID.SetReference(angle, rev::CANSparkMax::ControlType::kPosition);
 		return abs(angle - e_angle.GetPosition()) < tolerance;
 	}
	// void SetIntakeSpeed(float inPerMin) {
	// 	intakePID.SetReference(inPerMin, rev::CANSparkMax::ControlType::kVelocity);
	// }
    void SetIntake(float percent){
        m_intake.Set(percent/100);
    }
	// void SetShooterSpeed(float inPerMin) {
	// 	shooterPID1.SetReference(inPerMin, rev::CANSparkMax::ControlType::kVelocity);
	// 	shooterPID2.SetReference(inPerMin, rev::CANSparkMax::ControlType::kVelocity);
	// }
	// set all the motors required for shooting using percent output
	void SetShooter(float speed) {
		m1_shooter.Set(speed/100);
		m2_shooter.Set(speed/100);
	}
	int GetNotePresent() {
		return !eye0.Get() || !eye1.Get() || !eye2.Get(); // todo: get sensor value
	}
 	void init() {
		m_intake.RestoreFactoryDefaults();
        m_intake.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
		m_intake.SetInverted(true);
		intakePID.SetP(6e-5);
		intakePID.SetI(1e-6);
		intakePID.SetD(0);
		intakePID.SetFF(0.000015);
		e_intake.SetPositionConversionFactor(intakeIPR);
		m_intake.BurnFlash();

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
        m_angle.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
 		m_angle.SetInverted(true);
 		anglePID.SetP(0.1/angleDPR);
 		anglePID.SetI(0);
 		anglePID.SetD(6/angleDPR);
 		anglePID.SetFF(-0.05);
        anglePID.SetOutputRange(-0.25, 0.4);
 		e_angle.SetPositionConversionFactor(angleDPR);
 		m_angle.BurnFlash();

        m1_shooter.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        m2_shooter.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        ctre::phoenix6::configs::TalonFXConfiguration configs{};
        configs.CurrentLimits.StatorCurrentLimit = 100;

        m1_shooter.GetConfigurator().Apply(configs);
        m2_shooter.GetConfigurator().Apply(configs);
 	}
	
private:
	rev::CANSparkMax m_intake{42, rev::CANSparkMax::MotorType::kBrushless};
	rev::SparkPIDController intakePID = m_intake.GetPIDController();
	rev::SparkRelativeEncoder e_intake = m_intake.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	ctre::phoenix6::hardware::TalonFX m1_shooter{43, "CTREdevices"};

	ctre::phoenix6::hardware::TalonFX m2_shooter{44, "CTREdevices"};

 	rev::CANSparkMax m_angle{41, rev::CANSparkMax::MotorType::kBrushless};
 	rev::SparkPIDController anglePID = m_angle.GetPIDController();
 	rev::SparkRelativeEncoder e_angle = m_angle.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	frc::DigitalInput eye0{0};
	frc::DigitalInput eye1{1};
    frc::DigitalInput eye2{2};
	
	const float intakeGearboxReduction = 9;
	// inches per rotation of the intake motor
	const float intakeIPR = M_PI*2/intakeGearboxReduction;
	// inches per rotation of the shooter motors
	const float shooterIPR = M_PI*4;

const float angleGearboxReduction = 60;
// degrees per rotation of the angle motor
const float angleDPR = 360/angleGearboxReduction;
};
