// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>

class IntakeShooter {
public:
 	bool SetAngle(float angle, float tolerance = 4) {
 		anglePID.SetReference(angle, rev::CANSparkMax::ControlType::kPosition);
 		return abs(angle - e_angle.GetPosition()) < tolerance;
 	}
	void SetIntakeSpeed(float inPerSec) {
		intakePID.SetReference(inPerSec/60, rev::CANSparkMax::ControlType::kVelocity);
	}
    void SetIntake(float percent){
        m_intake.Set(percent/100);
    }
	void SetShooterSpeed(float inPerSec) { 
		auto friction_torque = (inPerSec > 0) ? 3_A : -3_A; // To account for friction, we add this to the arbitrary feed forward
		/* Use torque velocity */
		m1_shooter.SetControl(s_velocity.WithVelocity(inPerSec/shooterIPR*1_tps).WithFeedForward(friction_torque));
		m2_shooter.SetControl(s_velocity.WithVelocity(inPerSec/shooterIPR*1_tps).WithFeedForward(friction_torque));
	}
	void SetShooter(float speed) {
		m1_shooter.Set(speed/100);
		m2_shooter.Set(speed/100);
	}
	int GetNotePresent() {
		return !eye0.Get() || !eye1.Get() || eye2.Get(); // todo: get sensor value
	}
	double GetAngle() {
		return e_abs_angle.GetDistance();
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
		
 		m_angle.RestoreFactoryDefaults();
        m_angle.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
 		m_angle.SetInverted(true);
 		anglePID.SetP(0.1/angleDPR);
 		anglePID.SetI(0);
 		anglePID.SetD(6/angleDPR);
 		anglePID.SetFF(-0.05);
        anglePID.SetOutputRange(-0.25, 0.4);
 		e_angle.SetPositionConversionFactor(angleDPR);
		e_abs_angle.SetDistancePerRotation(360.0);
 		m_angle.BurnFlash();

        m1_shooter.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        m2_shooter.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        ctre::phoenix6::configs::TalonFXConfiguration configs{};
        configs.CurrentLimits.StatorCurrentLimit = 100;
		/* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
		configs.Slot1.kP = 8; // An error of 1 rotation per second results in 5 amps output
		configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
		configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

		configs.TorqueCurrent.PeakForwardTorqueCurrent = 60;  // Peak output of 40 amps
		configs.TorqueCurrent.PeakReverseTorqueCurrent = -60; // Peak output of 40 amps

        m1_shooter.GetConfigurator().Apply(configs);
        m2_shooter.GetConfigurator().Apply(configs);
 	}
	
private:
	rev::CANSparkMax m_intake{42, rev::CANSparkMax::MotorType::kBrushless};
	rev::SparkPIDController intakePID = m_intake.GetPIDController();
	rev::SparkRelativeEncoder e_intake = m_intake.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	ctre::phoenix6::controls::VelocityTorqueCurrentFOC s_velocity{0_tps, 0_tr_per_s_sq, 0_A, 1, false};
	ctre::phoenix6::hardware::TalonFX m1_shooter{43, "CTREdevices"};
	ctre::phoenix6::hardware::TalonFX m2_shooter{44, "CTREdevices"};

 	rev::CANSparkMax m_angle{41, rev::CANSparkMax::MotorType::kBrushless};
 	rev::SparkPIDController anglePID = m_angle.GetPIDController();
 	rev::SparkRelativeEncoder e_angle = m_angle.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	frc::DigitalInput eye0{0};
	frc::DigitalInput eye1{1};
    frc::DigitalInput eye2{2};
	frc::DutyCycleEncoder e_abs_angle{9};
	
	const float intakeGearboxReduction = 25;
	// inches per rotation of the intake motor
	const float intakeIPR = M_PI*2/intakeGearboxReduction;
	// inches per rotation of the shooter motors
	const float shooterIPR = M_PI*4;

const float angleGearboxReduction = 60;
// degrees per rotation of the angle motor
const float angleDPR = 360/angleGearboxReduction;
};
