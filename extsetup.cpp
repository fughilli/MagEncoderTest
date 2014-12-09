#include "extsetup.h"
#include "PIDControl.h"
#include <Wire.h>
#include <stdlib.h>
#include "Smoother.h"

#define SENSOR_ADDR 0x40
#define SENSOR_MAG_ADDR 0xFC
#define SENSOR_ANG_ADDR 0xFF
#define SENSOR_ZERO_ADDR 0x17

void setSensorZero(int);
int readSensor(char);
void setactuator(float);
float getscaledpos();
float calc_dt();
void initactuator();

float dt = 0;

float freq = 0.1f, ampl = 1.0f;
float laserf = 0.1f;

char ibuffer[12];
uint8_t ibufferIndex = 0;

template <class T>
T clamp(T val, T cmin, T cmax)
{
    return min(max(cmin, val), cmax);
}

int prev_millis = 0, cur_millis = 0;

int actuator_minpos, actuator_maxpos;

int accum = 0;

PIDController xcontroller(0.1f, 0.0f, 0.0f);
Smoother<float, 2> smooth;

void extsetup()
{
    Wire.begin();

    Serial.begin(115200);

    pinMode(3, OUTPUT);
    pinMode(5, OUTPUT);

    pinMode(2, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);

    digitalWrite(3, LOW);
    digitalWrite(5, LOW);

    pinMode(9, OUTPUT);

    //analogWrite(5, 128);

    Serial.println("Program start! Calibrate actuator...");
    delay(2000);

    setSensorZero(0);

    initactuator();

    Serial.print(actuator_minpos);
    Serial.print(", ");
    Serial.println(actuator_maxpos);
}

void setSensorZero(int zeropos)
{
    Wire.beginTransmission(SENSOR_ADDR);
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
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(SENSOR_ADDR, 2);
    while(Wire.available() < 2);

    int ret = (Wire.read()&0xFF)<<6;
    ret |= Wire.read() & 0x3F;

    return ret;
}

float calc_dt()
{
    cur_millis = millis();
    float ret = (cur_millis - prev_millis)/1000.0f;
    prev_millis = cur_millis;
    return ret;
}

void setactuator(float x)
{
    if(x == 0)
    {
        digitalWrite(5, LOW);
//        Serial.write('d');
    }
    if(x > 0)
    {
        if(x > 1.0f)
            x = 1.0f;
        digitalWrite(2, LOW);
        digitalWrite(6, HIGH);

        analogWrite(5, (uint8_t)(255 * x));
//        Serial.write('g');
    }
    if(x < 0)
    {
        if(x < -1.0f)
            x = -1.0f;
        digitalWrite(2, HIGH);
        digitalWrite(6, LOW);

        analogWrite(5, (uint8_t)(255 * -x));
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
    return (((float)(pos - actuator_minpos))/(actuator_maxpos - actuator_minpos))*2.0f - 1.0f;
}

void processCommand(char * buf)
{
    float * toset = 0;
    switch(buf[0])
    {
    case 'p':
    case 'P':
        toset = &xcontroller.coefficients.p;
        Serial.write('p');
        break;
    case 'i':
    case 'I':
        toset = &xcontroller.coefficients.i;
        Serial.write('i');
        break;
    case 'd':
    case 'D':
        toset = &xcontroller.coefficients.d;
        Serial.write('d');
        break;
    case 'f':
        toset = &freq;
        Serial.write('f');
        break;
    case 'g':
        Serial.println(dt, 10);
        return;
        break;
    case 'a':
        toset = &ampl;
        break;
    case 'l':
        toset = &laserf;
        break;
    }

    if(toset != 0)
    {
        *toset = atof(buf + 1);
        Serial.println(*toset, 10);
    }
}

float t = 0;

void extloop()
{
    dt = calc_dt();

    t += dt;

    float sint = ampl * sin(t*freq*TWO_PI);
    float lasersint = sin(t*laserf*TWO_PI);

    if(laserf > 0)
    {
        analogWrite(9, 255 * (lasersint/2.0f + 0.5f));
    }
    else
    {
        digitalWrite(9, (laserf < -1.0f)?HIGH:LOW);
    }

//    smooth.pushValue(getscaledpos());
//    float pos = smooth.pullValue();
    float pos = getscaledpos();

    float control = xcontroller.calculate(pos, sint, dt);

    setactuator(control);

    if(Serial.available())
    {
        ibuffer[ibufferIndex] = Serial.read();
        if(ibuffer[ibufferIndex++] == '\n')
        {
            processCommand(ibuffer);
            ibufferIndex = 0;
        }
    }

//    Serial.print(pos, 4);
//    Serial.print(",\t");
//    Serial.print(sint, 4);
//    Serial.print(",\t");
//    Serial.println(control, 4);

    //accum |= magnitude;
    //Serial.println(magnitude);
    //delay(10);
}

