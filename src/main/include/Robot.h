#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <complex.h>
#include <frc/Timer.h>

// #include "subsystems/Swerve.h"
#include "subsystems/IntakeShooter.h"
using namespace std;

class Robot : public frc::TimedRobot {
 public:
  int x = 0; // current autoPos setpoint index
  int i = 0; // next position index while waiting
  bool isShooting = false;
  struct autoValue{
    complex<float> pos;
    float angle = 0;
    units::time::second_t time = 0_s;
  };
  frc::XboxController controller{0};
  // Swerve swerve;
  IntakeShooter intakeShooter;
  autoValue autoPos[4] = {
    {complex<float>(15, 0), 0, 1_s},
    {complex<float>(15, 15), 0, 1_s},
    {complex<float>(0, 15), 0, 1_s},
    {complex<float>(0, 0), 0, 1_s}
  };
  frc::Timer posWaitTimer;
  frc::Timer shootTimer;
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
