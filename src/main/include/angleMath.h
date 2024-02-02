#pragma once

#include <math.h>

void limit(double &angle){
    while (angle > M_PI){
        angle -= M_PI*2;
    }
    while (angle < -M_PI){
        angle += M_PI*2;
    }
}