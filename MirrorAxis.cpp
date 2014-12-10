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
    //Serial.println("Clearing I2C input buffer...");
    while(Wire.available())
        Wire.read();

    //Serial.println("Reading slave...");
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(dev_addr, 2);

    long startmillis = millis();
    while(Wire.available() < 2)
    {
        if(millis() > startmillis + SENSOR_READ_TIMEOUT)
        {
            if(success != NULL)
                *success = false;
            return -1;
        }
    }

    int ret = (Wire.read()&0xFF)<<6;
    ret |= Wire.read() & 0x3F;

    if(success != NULL)
        *success = true;
    return ret;
}

void MirrorAxis::setActuatorPower(float x)
{
    Serial.write(6);
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
    Serial.write('7');
}

MirrorAxis::MirrorAxis(uint8_t sensor_addr, int dpin1, int dpin2, int drivepin) : PID(0.3f, 0.0f, 0.0001f)
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
        Serial.write('1');
        int pos = readSensor(m_dev_addr, SENSOR_ANG_ADDR, success);
        Serial.write('2');
        return m_pos = ((((float)(pos - m_actuator_minpos))/(m_actuator_maxpos - m_actuator_minpos))*2.0f - 1.0f);
    }
}

float MirrorAxis::getPos()
{
    return m_pos;
}



