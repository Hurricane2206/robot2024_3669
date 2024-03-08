#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/CANSparkMax.h>
#include <complex.h>
#include <string>
#include "angleMath.h"

using namespace std;

class Module{
public:
    Module(int modID, complex<float> pos){
        turnVector = pos*complex<float>(0, 1)/abs(pos);
        dMotor = new ctre::phoenix6::hardware::TalonFX(modID+10, "CTREdevices");
        encoder = new ctre::phoenix6::hardware::CANcoder(modID+20, "CTREdevices");
        current = new ctre::phoenix6::controls::TorqueCurrentFOC(0_A, 1, 0_A, true, false, false);
        sMotor = new rev::CANSparkMax(modID+30, rev::CANSparkMax::MotorType::kBrushless);
    }
    void init() {
        dMotor->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
        ctre::phoenix6::configs::TalonFXConfiguration configs{};

		/* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
		configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
		configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
		configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
		configs.TorqueCurrent.PeakForwardTorqueCurrent = 70;  // Peak output of 40 amps
		configs.TorqueCurrent.PeakReverseTorqueCurrent = -70; // Peak output of 40 amps

        dMotor->GetConfigurator().Apply(configs);
    }
    void set(complex<float> rVector, float turnRate){
        complex<float> modVector = getVelocity(rVector, turnRate);
        float throttle = abs(modVector);
        angle = encoder->GetAbsolutePosition().GetValue().value()*(M_PI*2);
        if (throttle > 0.001){
            float error = arg(modVector)-angle;
            am::limit(error);
            if (abs(error) > (M_PI/2)){
                error += M_PI;
                am::limit(error);
                throttle *= -1;
            }
            sMotor->Set(error/M_PI);
            auto friction_torque = (throttle > 0) ? 1_A : -1_A; // To account for friction, we add this to the arbitrary feed forward
            /* Use torque velocity */
            dMotor->SetControl(m_velocity.WithVelocity(throttle*90_tps).WithFeedForward(friction_torque));
        }
        else {
            dMotor->Set(0);
            dMotor->SetControl(m_velocity.WithVelocity(0_tps));
        }
    }
    complex<float> getPositionChange() {
        float motorPosChg = dMotor->GetPosition().GetValue().value() - motorPosOld;
        complex<float> modPosChange = polar<float>(motorPosChg*3.9*M_PI/6.12, angle);
        motorPosOld = dMotor->GetPosition().GetValue().value();
        return modPosChange;
    }

    float getMotorPos() {
        return dMotor->GetPosition().GetValue().value();
    }

    complex<float> getVelocity(complex<float> rVector, float turnRate){
        return rVector+turnVector*turnRate;
    }

private:
    ctre::phoenix6::hardware::TalonFX *dMotor;
    ctre::phoenix6::hardware::CANcoder *encoder;
	ctre::phoenix6::controls::VelocityTorqueCurrentFOC m_velocity{0_tps, 0_tr_per_s_sq, 0_A, 1, false};
    rev::CANSparkMax *sMotor;
    complex<float> turnVector;
    complex<float> modPosChange;
    float motorPosOld = 0;
    float angle;
};
/*

# # # # # # # #       # # # # # # # #
# C:11        # # # # #        C:13 #
# E:21                         E:23 #
# N:31                         N:33 #
#                                   #
#                                   #
#                                   #
#                                   #
#                                   #
#                                   #
#                                   #
#                                   #
#                                   #
#                                   #
# C:12                         C:14 #
# E:22                         E:24 #
# N:32                         N:34 #
# # # # # # # # # # # # # # # # # # #

*/
