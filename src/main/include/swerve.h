#pragma once

#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "SwerveModule.h"

using namespace std;

class Swerve{
public:
    void set(complex<double> velocity, double turnRate){
        angle = gyro.GetYaw()*-(M_PI/180);
        velocity *= polar<double>(1, -angle);
        double fastest = 1;
        for (Module module : modules){
            double speed = abs(module.getVelocity(velocity, turnRate));
            if (speed > fastest){
                fastest = speed;
            }
        }
        complex<double> posChange = complex<double>(0, 0);
        for (Module module : modules){
            module.set(velocity/fastest, turnRate/fastest);
            posChange += module.getPositionChange();
        }
        pos += posChange * polar<double>(0.25, angle);
        frc::SmartDashboard::PutNumber("posr", pos.real());
        frc::SmartDashboard::PutNumber("posi", pos.imag());
        frc::SmartDashboard::PutNumber("poschg", abs(posChange));
        frc::SmartDashboard::PutNumber("motposchg1", abs(modules[0].getPositionChange()));
        frc::SmartDashboard::PutNumber("motposchg2", abs(modules[1].getPositionChange()));
        frc::SmartDashboard::PutNumber("motposchg3", abs(modules[2].getPositionChange()));
        frc::SmartDashboard::PutNumber("motposchg4", abs(modules[3].getPositionChange()));
    }
    void setPos(complex<double> inputPos, double inputAngle){
        complex<double> posError = inputPos-pos;
        double angleError = inputAngle-angle;
        limit(angleError);
        complex<double> posPIDoutput = posError*0.01;
        double anglePIDoutput = angleError*0.3;
        if (abs(posPIDoutput) > 0.3) {
            posPIDoutput *= 0.3 / abs(posPIDoutput);
        }
        if (abs(anglePIDoutput) > 0.5) {
            anglePIDoutput *= 0.5 * abs(anglePIDoutput);
        }
        set(posPIDoutput, angleError/M_PI);
    }
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