/*
 * MirrorAxis.cpp
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

#include "MirrorAxis.h"
#include "Wire.h"
#include "Arduino.h"

static void setSensorZero(char dev_addr, int zeropos)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(SENSOR_ZERO_ADDR);
    Wire.write((zeropos >> 6)&0xFF);
    Wire.write(zeropos & 0x3F);
    Wire.endTransmission();
}

static int readSensor(char dev_addr, char reg_addr, bool * success)
{

    // Clear input buffer
    while(Wire.available())
    {
        Wire.read();
    }

    // Start slave read... write register address
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);

    // Write out buffer; check for error
    if(Wire.endTransmission(true) != 0)
    {
        if(success != NULL)
            *success = false;
        return -1;
    }

    // Request 16 bits
    Wire.requestFrom(dev_addr, 2);

    // Timeout if read fails
    uint32_t startmillis = millis();
    while(Wire.available() < 2)
    {
        if(millis() > startmillis + SENSOR_READ_TIMEOUT)
        {
            if(success != NULL)
                *success = false;
            return -1;
        }
    }

    // Concatenate
    int ret = (Wire.read()&0xFF)<<6;
    ret |= Wire.read() & 0x3F;

    if(success != NULL)
        *success = true;

    return ret;
}

void MirrorAxis::setActuatorPower(float x)
{
    if(x == 0)
    {
        digitalWrite(m_drivepin, LOW);
    }
    if(x > 0)
    {
        if(x > 1.0f)
            x = 1.0f;
        digitalWrite(m_dpin1, LOW);
        digitalWrite(m_dpin2, HIGH);

        analogWrite(m_drivepin, (uint8_t)(255 * x));
    }
    if(x < 0)
    {
        if(x < -1.0f)
            x = -1.0f;
        digitalWrite(m_dpin1, HIGH);
        digitalWrite(m_dpin2, LOW);

        analogWrite(m_drivepin, (uint8_t)(255 * -x));
    }
}

MirrorAxis::MirrorAxis(uint8_t sensor_addr, int dpin1, int dpin2, int drivepin) : PID(0.5f, 0.0f, 0.001f)
{
    m_dev_addr = sensor_addr;
    m_dpin1 = dpin1;
    m_dpin2 = dpin2;
    m_drivepin = drivepin;

    m_actuator_minpos = 0;
    m_actuator_maxpos = 0;

    m_pos = 0;
    m_setpos = 0;

    m_initfail = true;
}

void MirrorAxis::initialize()
{
    bool succ;

    // Reset the sensor
    setSensorZero(m_dev_addr, 0);

    pinMode(m_dpin1, OUTPUT);
    pinMode(m_dpin2, OUTPUT);

    pinMode(m_drivepin, OUTPUT);

    digitalWrite(m_drivepin, LOW);

    // Calibration procedure
    setActuatorPower(-1);
    Serial.println("Min pos");
    delay(2000);
    m_actuator_minpos = readSensor(m_dev_addr, SENSOR_ANG_ADDR, &succ);
    if(!succ)
    {
        m_initfail = true;
        return;
    }
    setActuatorPower(1);
    Serial.println("Max pos");
    delay(2000);
    m_actuator_maxpos = readSensor(m_dev_addr, SENSOR_ANG_ADDR, &succ);
    if(!succ)
    {
        m_initfail = true;
        return;
    }
    setActuatorPower(0);

    m_initfail = m_actuator_maxpos == m_actuator_minpos;
}

void MirrorAxis::update(float dt)
{
    if(!m_initfail)
    {
        bool success;
        _getPos(&success);
        if(!success)
            return;
        setActuatorPower(PID.calculate(m_pos, m_setpos, dt));
    }
}

void MirrorAxis::setPos(float pos)
{
    m_setpos = pos;
}

float MirrorAxis::_getPos(bool * success)
{
    if(!m_initfail)
    {
        int pos = readSensor(m_dev_addr, SENSOR_ANG_ADDR, success);
        return m_pos = ((((float)(pos - m_actuator_minpos))/(m_actuator_maxpos - m_actuator_minpos))*2.0f - 1.0f);
    }
    return 0;
}

float MirrorAxis::getPos()
{
    return m_pos;
}



