#pragma once
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <string>
#include <complex>

using namespace std;

class Limelight {
public:
    float tx;
    float ty;
    float pitch;
    bool targetValid;

    void update() {
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        tx = table->GetNumber("tx", 0.0);
        ty = table->GetNumber("ty", 0.0);
        targetValid = table->GetNumber("tx", 0.0) != 0;
        pitch = 0.0038*pow(ty, 2)+0.6508*ty+65.3899;
    }

private:
    std::shared_ptr<nt::NetworkTable> table;
} ll;