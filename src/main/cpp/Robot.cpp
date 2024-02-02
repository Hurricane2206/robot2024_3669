// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "math.h"

using namespace std;

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {
  swerve.setPos(complex<double>(0, 0), 0);
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  complex<double> velocity = complex<double>(-controller.GetLeftY(), -controller.GetLeftX());
  double turnRate = -controller.GetRightX()*0.5;
  swerve.set(velocity, turnRate);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
