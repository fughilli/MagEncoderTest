#include "PIDControl.h"
#include "Arduino.h"
#include "HardwareSerial.h"

PIDController::PIDController(float pCoeff, float iCoeff, float dCoeff, float iDecayFactor)
{
    coefficients.p = pCoeff;
    coefficients.i = iCoeff;
    coefficients.d = dCoeff;
    coefficients.idf = iDecayFactor;
    state.integral = 0.0f;
    state.prev_error_x = 0.0f;
}

float PIDController::calculate(float x, float target_x, float dt)
{
    float error_x = (target_x - x);
    float P = (error_x * coefficients.p);

    state.integral += (error_x * dt);
    float I = (state.integral * coefficients.i);
    state.integral/=coefficients.idf;

    if(abs(state.integral) < I_ZERO_THRESHOLD)
    {
        state.integral = 0.0f;
    }

    float D = ((error_x - state.prev_error_x) / dt * coefficients.d);

    state.prev_error_x = error_x;

    return P + I + D;
}
