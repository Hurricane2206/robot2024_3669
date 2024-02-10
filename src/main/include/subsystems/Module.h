// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "rev/CANSparkMax.h"
#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "complex"
#include "angleMath.h"
using namespace std;

class Module
{
public:
	Module(int ID, complex<float> pos) {
        m_drive = new ctre::phoenix6::hardware::TalonFX(10+ID, "rio");
        encoder = new ctre::phoenix6::hardware::CANcoder{20+ID, "rio"};
        m_twist = new rev::CANSparkMax(30+ID, rev::CANSparkMax::MotorType::kBrushless);
        turnVector = pos*complex<float>(0, 1)/abs(pos);
    }
	void set(complex<float> driveV, float turnV) {
        complex<float> V = getV(driveV, turnV);
        float m_driveV = abs(V);
        if (m_driveV > 0.001) {
            float error = arg(V) - encoder->GetAbsolutePosition().GetValue().value()*2*M_PI;
            wrap(error);
            if (abs(error) > M_PI_2) {
                error += M_PI;
                wrap(error);
                m_driveV = -m_driveV;
            }
            m_drive->Set(m_driveV);
            m_twist->Set(error/M_PI);
        } else {
            m_drive->Set(0);
            m_twist->Set(0);
        }
    }
    complex<float> getV(complex<float> dV, float tV) {
        return dV + turnVector * tV;
    }


private:
	ctre::phoenix6::hardware::TalonFX *m_drive;
    ctre::phoenix6::hardware::CANcoder *encoder;
    rev::CANSparkMax *m_twist;
    complex<float> turnVector;
};
