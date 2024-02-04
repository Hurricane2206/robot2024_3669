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

    Module(int modID, complex<double> pos){
        turnVector = pos*complex<double>(0, 1)/abs(pos);
        dMotor = new ctre::phoenix6::hardware::TalonFX(modID+10, "rio");
        encoder = new ctre::phoenix6::hardware::CANcoder(modID+20, "rio");
        sMotor = new rev::CANSparkMax(modID+30, rev::CANSparkMax::MotorType::kBrushless);
    }
    void set(complex<double> rVector, double turnRate){
        complex<double> modVector = getVelocity(rVector, turnRate);
        double throttle = abs(modVector);
        double angle = encoder->GetAbsolutePosition().GetValueAsDouble()*(M_PI*2);
        if (throttle > 0.001){
            double error = arg(modVector)-angle;
            limit(error);
            if (abs(error) > (M_PI/2)){
                error += M_PI;
                limit(error);
                throttle *= -1;
            }
            sMotor->Set(error/M_PI);
            dMotor->Set(throttle);
        }
        else {
            dMotor->Set(0);
            sMotor->Set(0);
        }
        motorPos = dMotor->GetPosition().GetValue().value();
        motorPosChg = motorPos-motorPosOld;
        modPosChange = complex<double>(cos(angle)*motorPosChg*3.9*M_PI/6.75, sin(angle)*motorPosChg*3.9*M_PI/6.75);
        motorPosOld = motorPos;
    }

    complex<double> getPositionChange() {
        return modPosChange;
    }

    double getMotPosChg() {
        return motorPosChg;
    }

    double getMotorPos() {
        return motorPos;
    }

    complex<double> getVelocity(complex<double> rVector, double turnRate){
        return rVector+turnVector*turnRate;
    }

private:
    ctre::phoenix6::hardware::TalonFX *dMotor;
    ctre::phoenix6::hardware::CANcoder *encoder;
    rev::CANSparkMax *sMotor;
    complex<double> turnVector;
    complex<double> modPosChange;
    double motorPos;
    double motorPosOld = 0;
    double motorPosChg;
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