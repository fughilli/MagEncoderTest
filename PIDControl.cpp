/*
 * PIDControl.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Kevin Balke
 *
 *
 * Copyright (C) 2012-2014  Kevin Balke, Jacob Sharf
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

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
