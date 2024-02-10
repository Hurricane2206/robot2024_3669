#pragma once

#include "intake.h"
#include "arm.h"

class Manipulator{
public:
    void shoot(){}
    void load(){}
    void unload(){}
private:
    Arm arm;
    Intake intake;
};