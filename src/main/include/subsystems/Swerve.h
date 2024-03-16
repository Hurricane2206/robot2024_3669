#pragma once

#include <AHRS.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SwerveModule.h"

using namespace std;

class Swerve{
public:
    complex<float> pos = complex<float>(0, 0);
    complex<float> posChange = complex<float>(0, 0);
    float angle;

    complex<float> posSetpoint = complex<float>(0, 0);
    complex<float> posError = complex<float>(0, 0);
    float posP = 0.003;
    // float angleSetpoint;

    complex<float> currentVelocity;
    float currentTurnRate = 0;
    complex<float> targetVelocity;

    const float slewRate = 0.04;
    void set(complex<float> velocity, float turnRate, bool noAcceleration = false){
        angle = -gyro.GetYaw()*(M_PI/180);
        targetVelocity = velocity;

        // robot orient the velocity
        velocity *= polar<float>(1, -angle);

        // find fastest module speed
        float fastest = 1;
        for (Module module : modules){
            float speed = abs(module.getVelocity(velocity, turnRate));
            if (speed > fastest){
                fastest = speed;
            }
        }
        // move current velocity toward target
        targetVelocity /= fastest;
        turnRate /= fastest;
        complex<float> velError = targetVelocity-currentVelocity;
        float turnRateError = turnRate - currentTurnRate;
        if (abs(velError) > slewRate) {
            velError *= slewRate/abs(velError);
        }
        if (abs(turnRateError) > slewRate) {
            turnRateError *= slewRate/abs(turnRateError);
        }
        currentVelocity += velError;
        currentTurnRate += turnRateError;
        targetVelocity = currentVelocity * polar<float>(1, -angle);

        // calculate odometry
        posChange = complex<float>(0, 0);
        for (Module module : modules) {
            module.set(targetVelocity, currentTurnRate);
            posChange += module.getPositionChange();
            // module.getPositionChange();
            // module.getPositionChange();
        }
        pos += posChange * polar<float>(0.25, angle);
    }

    void SetPosition(complex<float> position){
        posSetpoint = position;
    }

    bool GetPositionReached(float tolerance = 1) {
        return abs(posError) < tolerance;
    }

    // drive toward the position setpoint
    void RunPID(double tx) {
        // calculate PID Response
        angle = -gyro.GetYaw()*(M_PI/180);
        posError = posSetpoint-pos;
        complex<float> posPIDoutput = posP*posError;
        float turnRate = -tx / 100.0;
        if (abs(posPIDoutput) > 0.3) {
            posPIDoutput *= 0.3 / abs(posPIDoutput);
        }
        if (abs(turnRate) > 0.3) {
            turnRate *= 0.3 / abs(turnRate);
        }
        posPIDoutput *= polar<float>(1, -angle);
        // calculate odometry
        posChange = complex<float>(0, 0);
        for (Module module : modules){
            module.set(posPIDoutput, turnRate);
            posChange += module.getPositionChange();
            // module.getPositionChange();
            // module.getPositionChange();
        }
        pos += posChange * polar<float>(0.25, angle);
    }

    void init(){
        for (Module module : modules){
            module.init();
        }
    }

    void resetPos() {
        for (Module module : modules) {
            module.resetEncoders();
            module.resetEncoders();
        }
        pos = complex<float>(0,0);
        pos = complex<float>(0,0);
    }

    complex<float> GetModulePosChange(int i) {
        return modules[i].getPositionChange();
    }

    float GetMotorPosChange(int i) {
        return modules[i].getMotorPosChange();
    }

    float GetMotorPos(int i) {
        return modules[i].getMotorPos();
    }

private:
    AHRS gyro{frc::SPI::Port::kMXP};
    Module modules[4] = {
        Module{1, complex<float>(1, 1)},
        Module{2, complex<float>(-1, 1)},
        Module{3, complex<float>(1, -1)},
        Module{4, complex<float>(-1, -1)}
    };

} swerve;
