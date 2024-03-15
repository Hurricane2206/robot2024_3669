#pragma once;
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

enum TeleopState teleopState = TeleopState::DEFAULT;
enum TeleopState autoState = TeleopState::DEFAULT;
enum TeleopState lastTeleopState = TeleopState::DEFAULT;
enum TeleopState lastAutoState = TeleopState::DEFAULT;

// Array of function pointers for state initialization code
void (*init[TeleopState::NUM_OF_STATES])();
// Array of function pointers for the state periodic code
void (*periodic[TeleopState::NUM_OF_STATES])();

void defineTeleopStateFunctions();