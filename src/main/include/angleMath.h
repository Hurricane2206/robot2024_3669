#pragma once

#include <math.h>
namespace am{
    void limit(float &angle){
        while (angle > M_PI){
            angle -= M_PI*2;
        }
        while (angle < -M_PI){
            angle += M_PI*2;
        }
    }
}