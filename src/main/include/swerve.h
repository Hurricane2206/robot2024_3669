#pragma once

#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "SwerveModule.h"

using namespace std;

class Swerve{
public:
    void set(complex<double> velocity, double turnRate){
        double fastest = 1;
        angle = gyro.GetYaw()*-(M_PI/180);
        velocity *= polar<double>(1, -angle);
        for (Module module : modules){
            double speed = abs(module.getVelocity(velocity, turnRate));
            if (speed > fastest){
                fastest = speed;
            }
        }
        // complex<double> posChange = complex<double>(0, 0);
        for (Module module : modules){
            module.set(velocity/fastest, turnRate/fastest);
            // posChange += module.getPosChange();
        }
        // pos += posChange*polar<double>(0.25, angle);
        frc::SmartDashboard::PutNumber("motorPosChg", modules[0].motorPosChg);
        // frc::SmartDashboard::PutNumber("Imaginary", modules[0].motorPosChange.imag());
        frc::SmartDashboard::PutNumber("Motor1", modules[0].motorPos);
    }
    // void setPos(complex<double> inputPos, double inputAngle){
    //     complex<double> posError = inputPos-pos;
    //     double angleError = inputAngle-angle;
    //     limit(angleError);
    //     complex<double> posPIDoutput = posError*0.01;
    //     if (abs(posPIDoutput) > 1){
    //         posPIDoutput /= abs(posPIDoutput);
    //     }
    //     set(posPIDoutput, angleError/M_PI);
    // }
private:
    AHRS gyro{frc::SPI::Port::kMXP};
    Module modules[4] = {
        Module{1, complex<double>(25, 17.75)},
        Module{2, complex<double>(-25, 17.75)},
        Module{3, complex<double>(25, -17.75)},
        Module{4, complex<double>(-25, -17.75)}
    };
    complex<double> pos = complex<double>(0, 0);
    double angle;
};