/*
 * MirrorAxis.h
 *
 *  Created on: Dec 8, 2014
 *      Author: Kevin Balke
 *
 *
 * Copyright (C) 2012-2014  Kevin Balke, Iyal Suresh, Alexander Fong
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

#ifndef _MIRROR_AXIS_H_
#define _MIRROR_AXIS_H_

#include "PIDControl.h"

#define SENSOR_MAG_ADDR 0xFC
#define SENSOR_ANG_ADDR 0xFF
#define SENSOR_ZERO_ADDR 0x17

#define SENSOR_READ_TIMEOUT 2

class MirrorAxis
{
public:
    PIDController PID;

    MirrorAxis(uint8_t sensor_addr, int dpin1, int dpin2, int drivepin);

    void initialize();
    void update(float dt);
    void setPos(float pos);
    float getPos();
private:
    float _getPos(bool * success = NULL);
    void setActuatorPower(float x);
    int m_dpin1, m_dpin2, m_drivepin;
    uint8_t m_dev_addr;
    float m_pos, m_setpos;
    int m_actuator_minpos, m_actuator_maxpos;
    bool m_initfail;
};

#endif // _MIRROR_AXIS_H_
