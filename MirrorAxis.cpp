#include "MirrorAxis.h"
#include "Wire.h"

static void setSensorZero(uint8_t dev_addr, int zeropos)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(SENSOR_ZERO_ADDR);
    Wire.write((zeropos >> 6)&0xFF);
    Wire.write(zeropos & 0x3F);
    Wire.endTransmission();
}

static int readSensor(uint8_t dev_addr, uint8_t reg_addr)
{
    //Serial.println("Clearing I2C input buffer...");
    while(Wire.available())
        Wire.read();

    //Serial.println("Reading slave...");
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(dev_addr, 2);
    while(Wire.available() < 2);

    int ret = (Wire.read()&0xFF)<<6;
    ret |= Wire.read() & 0x3F;

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

MirrorAxis::MirrorAxis(uint8_t sensor_addr, int dpin1, int dpin2, int drivepin) : PID(0.5f, 0.0f, 0.0f)
{
    m_dev_addr = sensor_addr;
    m_dpin1 = dpin1;
    m_dpin2 = dpin2
    m_drivepin = drivepin;

    m_actuator_minpos = 0;
    m_actuator_maxpos = 0;

    m_pos = 0;
    m_setpos = 0;
}

void MirrorAxis::initialize()
{
    // Reset the sensor
    setSensorZero(m_dev_addr, 0);

    pinMode(m_dpin1, OUTPUT);
    pinMode(m_dpin2, OUTPUT);

    pinMode(m_drivepin, OUTPUT);

    digitalWrite(m_drivepin, LOW);

    // Calibration procedure
    setActuatorPower(-0.5);
    Serial.println("Min pos");
    delay(1000);
    m_actuator_minpos = readSensor(m_dev_addr, SENSOR_ANG_ADDR);
    setActuatorPower(0.5);
    Serial.println("Max pos");
    delay(1000);
    m_actuator_maxpos = readSensor(m_dev_addr, SENSOR_ANG_ADDR);
    setActuatorPower(0);
}

void MirrorAxis::update(float dt)
{
    setActuatorPower(PID.calculate(_getPos(), m_setpos, dt))
}

void MirrorAxis::setPos(float pos)
{
    m_setpos = pos;
}

float MirrorAxis::_getPos()
{
    int pos = readSensor(m_dev_addr, SENSOR_ANG_ADDR);
    return m_pos = ((((float)(pos - m_actuator_minpos))/(m_actuator_maxpos - m_actuator_minpos))*2.0f - 1.0f);
}

float MirrorAxis::getPos()
{
    return m_pos;
}



