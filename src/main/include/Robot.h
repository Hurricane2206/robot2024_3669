#pragma once

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <complex.h>
#include <frc/Timer.h>
#include <string.h>

#include "subsystems/Swerve.h"
#include "subsystems/IntakeShooter.h"
#include "subsystems/NoteHandler.h"
#include "subsystems/Climb.h"
using namespace std;

class Robot : public frc::TimedRobot{
public:
	int x = 0; // current autoPos setpoint index
	int i = 0; // next position index while waiting

	enum State {
		IDLE,
		ARMDEFAULT,
		INTAKING,
		AIMING,
		RAMPING,
		SHOOTING,
		AMPTRANSFER,
		TRAPTRANSFER,
		INTAKECLEAR,
		TRAPCLIMBUP,
		TRAPCLIMBDOWN,
		TRAPSCOREREADY,
		WAITINGTOSCORE,
		TRAPSCORE,
		CLIMBUP,
		CLIMBDOWN,
		AMPSCORE,
		AMPOS,
		WAITFORCLIMB
	};
	enum State robotState = IDLE;
	enum State lastRobotState = IDLE;
	
	struct autoValue
	{
		complex<float> pos;
		float angle = 0;
		units::time::second_t time = 0_s;
	};
	frc::XboxController controller{0};
	frc::Joystick key_pad{1};
	Swerve swerve;
	NoteHandler arm;
	IntakeShooter intakeShooter;
	Climb climb;
	autoValue autoPos[4] = {
		{complex<float>(15, 0), 0, 1_s},
		{complex<float>(15, 15), 0, 1_s},
		{complex<float>(0, 15), 0, 1_s},
		{complex<float>(0, 0), 0, 1_s}};
	frc::Timer posWaitTimer;
	frc::Timer timer;
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
