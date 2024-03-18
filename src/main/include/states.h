#pragma once

#include <complex.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Swerve.h"
#include "subsystems/IntakeShooter.h"
#include "subsystems/NoteHandler.h"
#include "subsystems/Climb.h"
#include "subsystems/Limelight.h"

using namespace std;

enum AutoState {
    ADRIVING,
    AINTAKING,
    AAIMING,
    ARAMPING,
    ASHOOTING,
    ANOTEALIGN1,
    ANOTEALIGN2,
    ANOTEALIGN3,

    ANUM_OF_STATES // the number of states: do not remove
};
enum TeleopState {
    DEFAULT,
    DEFENDING,
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
    WAITFORCLIMB,

    NUM_OF_STATES // the number of states: do not remove
};

enum AutoState autoState = AutoState::ADRIVING;
enum AutoState lastAutoState = AutoState::ADRIVING;
enum TeleopState teleopState = TeleopState::DEFAULT;
enum TeleopState lastTeleopState = TeleopState::DEFAULT;

frc::Timer timer;
frc::XboxController controller{0};
frc::Joystick key_pad{1};

struct autoValue
{
	complex<float> pos;
	float angle = 0;
	enum AutoState startingState = AutoState::ADRIVING;
};

autoValue autoPose[3] = {
	{complex<float>(0, 0), 0, AutoState::AAIMING},
	{complex<float>(60,0), 0, AutoState::AINTAKING},
	{complex<float>(30,0), 0, AutoState::AAIMING}
};

unsigned int x = 0; // current autoPos setpoint index

// Array of function pointers for state initialization code
void (*AutoInit[AutoState::ANUM_OF_STATES])();
// Array of function pointers for the state periodic code
void (*AutoPeriodic[AutoState::ANUM_OF_STATES])();

void defineAutoStateFunctions();

// Array of function pointers for state initialization code
void (*TelInit[TeleopState::NUM_OF_STATES])();
// Array of function pointers for the state periodic code
void (*TelPeriodic[TeleopState::NUM_OF_STATES])();

void defineTeleopStateFunctions();

