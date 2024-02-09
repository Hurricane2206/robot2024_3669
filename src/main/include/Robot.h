#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <complex.h>

#include "Swerve.h"

class Robot : public frc::TimedRobot {
 public:
  int x = 0;
  struct autoValue{
    complex<double> pos;
    double angle;
  };
  frc::XboxController controller{0};
  Swerve swerve;
  autoValue autoPos[4] = {
    {complex<double>(15, 0), 0},
    {complex<double>(15, 15), 0},
    {complex<double>(0, 15), 0},
    {complex<double>(0, 0), 0}
  };
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
};
