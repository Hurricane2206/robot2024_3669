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
#include "subsystems/Limelight.h"
using namespace std;

class Robot : public frc::TimedRobot{
public:
	unsigned int x = 0; // current autoPos setpoint index
	float pitch = 40;
	float ty;

	enum State {
		DEFAULT,
		ARMDEFAULT,
		INTAKING,
		NOTEALIGN1,
		NOTEALIGN2,
		NOTEALIGN3,
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
	enum State robotState = DEFAULT;
	enum State lastRobotState = DEFAULT;
	
	struct autoValue
	{
		complex<float> pos;
		float angle = 0;
		enum State setpointState = DEFAULT;
		float shooterPitch = 75;
	};
	frc::XboxController controller{0};
	frc::Joystick key_pad{1};
	Swerve swerve;
	NoteHandler arm;
	IntakeShooter intakeShooter;
	Limelight ll;
	Climb climb;
	autoValue autoPose[4] = {
		{complex<float>(0, 0), 0, AIMING, 75}};
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
