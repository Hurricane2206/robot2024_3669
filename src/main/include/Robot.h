#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <complex.h>

#include "Swerve.h"

class Robot : public frc::TimedRobot {
 public:
  frc::XboxController controller{0};
  Swerve swerve;

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
