#pragma once
#include "LimelightHelpers.h"
#include <string>
#include <complex>

using namespace std;

class Limelight {
public:
    double getSpeakerYaw() {
        LimelightHelpers::setPipelineIndex("", 1);
        return LimelightHelpers::getTX("");
    }

    // get the position of the robot in inches
    complex<double> getPosition() {
        LimelightHelpers::setPipelineIndex("", 0);
        return complex<float>(LimelightHelpers::getBotpose_TargetSpace()[0], LimelightHelpers::getBotpose_TargetSpace()[1]) * 39.37f;
    }

    // 
};