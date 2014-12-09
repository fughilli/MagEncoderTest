#ifndef PIDCONTROL_H
#define PIDCONTROL_H

//#include <cmath>
#include "Arduino.h"

#define I_ZERO_THRESHOLD 0.001

class PIDController
{
private:
    struct PIDControlState_t
    {
        float integral;
        float prev_error_x;
    } state;
public:
    struct PIDControlCoefficients_t
    {
        float p, i, d;
        float idf;
    } coefficients;
    PIDController(float pCoeff, float iCoeff, float dCoeff, float iDecayFactor = 2.0f);
    float calculate(float x, float target_x, float dt);
};

#endif // PIDCONTROL_H
