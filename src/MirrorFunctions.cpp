#include "MirrorFunctions.h"

#define SENSOR_MAG_ADDR 0xFC
#define SENSOR_ANG_ADDR 0xFF
#define SENSOR_ZERO_ADDR 0x17

MirrorFunctions::MirrorFunctions(uint8_t sensorAddress, int drivePin, int leftPin, int rightPin) : m_pidl(0.1f, 0.0f, 0.0f)
{
    m_sensorAddress = sensorAddress;
    m_drivePin = drivePin;
    m_leftPin = leftPin;
    m_rightPin = rightPin;

}

MirrorFunctions::~MirrorFunctions()
{
    delete m_pidl;
}

void setSensorZero(int zeropos)
{
    Wire.beginTransmission(m_sensorAddress);
    Wire.write(SENSOR_ZERO_ADDR);
    Wire.write((zeropos >> 6)&0xFF);
    Wire.write(zeropos & 0x3F);
    Wire.endTransmission();
}

int readSensor(char reg_addr)
{
    //Serial.println("Clearing I2C input buffer...");
    while(Wire.available())
        Wire.read();

    //Serial.println("Reading slave...");
    Wire.beginTransmission(m_sensorAddress);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(m_sensorAddress, 2);
    while(Wire.available() < 2);

    int ret = (Wire.read()&0xFF)<<6;
    ret |= Wire.read() & 0x3F;

    return ret;
}

void setactuator(float x)
{
    if(x == 0)
    {
        digitalWrite(m_drivePin, LOW);
//        Serial.write('d');
    }
    if(x > 0)
    {
        if(x > 1.0f)
            x = 1.0f;
        digitalWrite(m_leftPin, LOW);
        digitalWrite(m_rightPin, HIGH);

        analogWrite(m_drivePin, (uint8_t)(255 * x));
//        Serial.write('g');
    }
    if(x < 0)
    {
        if(x < -1.0f)
            x = -1.0f;
        digitalWrite(m_leftPin, HIGH);
        digitalWrite(m,_rightPin, LOW);

        analogWrite(m_drivePin, (uint8_t)(255 * -x));
//        Serial.write('l');
    }
}

void initactuator()
{
    setactuator(-0.5);
    Serial.println("Min pos");
    delay(2000);
    actuator_minpos = readSensor(SENSOR_ANG_ADDR);
    setactuator(0.5);
    Serial.println("Max pos");
    delay(2000);
    actuator_maxpos = readSensor(SENSOR_ANG_ADDR);
    setactuator(0);
}

float getscaledpos()
{
    int pos = readSensor(SENSOR_ANG_ADDR);
    return (((float)(pos - m_actuator_minpos))/(m_actuator_maxpos - m_actuator_minpos))*2.0f - 1.0f;
}

void calculate(float x, float target_x, float dt){
    return m_pidl.calculate(x,target_x,dt);
}
