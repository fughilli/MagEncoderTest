/*
 * PIDControl.h
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
