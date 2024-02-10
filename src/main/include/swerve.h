#pragma once

#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "SwerveModule.h"

using namespace std;

class Swerve{
public:
    void set(complex<float> velocity, float turnRate){
        angle = gyro.GetYaw()*-(M_PI/180);
        velocity *= polar<float>(1, -angle);
        float fastest = 1;
        for (Module module : modules){
            float speed = abs(module.getVelocity(velocity, turnRate));
            if (speed > fastest){
                fastest = speed;
            }
        }
        complex<float> posChange = complex<float>(0, 0);
        for (Module module : modules){
            module.set(velocity/fastest, turnRate/fastest);
            posChange += module.getPositionChange();
        }
        pos += posChange * polar<float>(0.25, angle);
        frc::SmartDashboard::PutNumber("posr", pos.real());
        frc::SmartDashboard::PutNumber("posi", pos.imag());
        frc::SmartDashboard::PutNumber("poschg", abs(posChange));
        frc::SmartDashboard::PutNumber("mchg1", abs(modules[0].getPositionChange()));
        frc::SmartDashboard::PutNumber("mchg2", abs(modules[1].getPositionChange()));
        frc::SmartDashboard::PutNumber("mchg3", abs(modules[2].getPositionChange()));
        frc::SmartDashboard::PutNumber("mchg4", abs(modules[3].getPositionChange()));
    }
    bool setPos(complex<float> inputPos, float inputAngle){
        complex<float> posError = inputPos-pos;
        float angleError = inputAngle-angle;
        am::limit(angleError);
        complex<float> posPIDoutput = posError*polar<float>(0.025, 1);
        float anglePIDoutput = angleError*0.2;
        if (abs(posPIDoutput) > 0.3) {
            posPIDoutput *= 0.3 / abs(posPIDoutput);
        }
        if (abs(anglePIDoutput) > 0.5) {
            anglePIDoutput *= 0.5 * abs(anglePIDoutput);
        }
        set(posPIDoutput, angleError);
        return abs(posError) < 1;
    }
    void zeroPos(){
        for (Module module : modules){
            module.zero();
            pos *= 0;
        }
    }
private:
    AHRS gyro{frc::SPI::Port::kMXP};
    Module modules[4] = {
        Module{1, complex<float>(25, 17.75)},
        Module{2, complex<float>(-25, 17.75)},
        Module{3, complex<float>(25, -17.75)},
        Module{4, complex<float>(-25, -17.75)}
    };
    complex<float> pos = complex<float>(0, 0);
    float angle;
};