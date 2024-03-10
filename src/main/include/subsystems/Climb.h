#pragma once

#include <rev/CANSparkMax.h>

class Climb {
public:
    bool SetHeight(float height, float tolerance = 0.5) {
		elevatorPID.SetReference(height, rev::CANSparkMax::ControlType::kPosition);
		return abs(height - e_elevator.GetPosition()) < tolerance;
	}

    // void SetSpeed(float speed) {
    //     m_elevator.Set(speed);
    //     m2_elevator.Set(speed);
    // }

    void init() {
		m_elevator.RestoreFactoryDefaults();
		m_elevator.SetInverted(true);
		m_elevator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
		elevatorPID.SetP(0.12/elevatorIPR);
		elevatorPID.SetI(0);
		elevatorPID.SetD(1/elevatorIPR);
		elevatorPID.SetFF(0.000156);
		elevatorPID.SetOutputRange(-0.1, 0.1);
		e_elevator.SetPositionConversionFactor(elevatorIPR);
		m_elevator.BurnFlash();

		m2_elevator.RestoreFactoryDefaults();
		// m2_elevator.SetInverted(true);
		// m2_elevator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
		// elevatorPID.SetP(0.12/elevatorIPR);
		// elevatorPID.SetI(0);
		// elevatorPID.SetD(1/elevatorIPR);
		// elevatorPID.SetFF(0.000156);
		// elevatorPID.SetOutputRange(-0.5, 0.5);
		// m2_elevator.BurnFlash();
        m2_elevator.Follow(m_elevator);
    }


private:
	rev::CANSparkMax m_elevator{61, rev::CANSparkMax::MotorType::kBrushless};
	rev::SparkPIDController elevatorPID = m_elevator.GetPIDController();
	rev::SparkRelativeEncoder e_elevator = m_elevator.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::CANSparkMax m2_elevator{62, rev::CANSparkMax::MotorType::kBrushless};
	// rev::SparkPIDController elevatorPID2 = m2_elevator.GetPIDController();
	// elevator gearbox reduction
	const float elevatorGearboxReduction = 36;
	// inches per rotation of the motor
	const float elevatorIPR = 9/2.54/elevatorGearboxReduction;
};