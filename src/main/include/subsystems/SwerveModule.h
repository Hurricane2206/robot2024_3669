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
        sMotor = new rev::CANSparkMax(modID+30, rev::CANSparkMax::MotorType::kBrushless);
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
            dMotor->Set(throttle);
        }
        else {
            dMotor->Set(0);
            sMotor->Set(0);
        }
    }
    void zero(){
        dMotor->SetPosition(0_tr);
        motorPosOld = 0;
    }
    complex<float> getPositionChange() {
        float motorPosChg = dMotor->GetPosition().GetValue().value() - motorPosOld;
        complex<float> modPosChange = polar<float>(motorPosChg*3.9*M_PI/6.75, angle);
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