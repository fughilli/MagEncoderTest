#include "extsetup.h"
#include "PIDControl.h"
#include <Wire.h>
#include <stdlib.h>
#include "Smoother.h"
#include "MirrorAxis.h"

float calc_dt();

float dt = 0;

float freq1 = 0.1f, freq2 = 0.1f, ampl1 = 1.0f, ampl2 = 1.0f, phas1 = 0.0f, phas2 = 0.0f;
float laserf = 0.1f;

char ibuffer[32];
uint8_t ibufferIndex = 0;

template <class T>
T clamp(T val, T cmin, T cmax)
{
    return min(max(cmin, val), cmax);
}

int prev_millis = 0, cur_millis = 0;

int accum = 0;

MirrorAxis MAYaw(0x40, 6, 2, 5), MAPitch(0x41, 7, 4, 3);

void extsetup()
{
    Wire.begin();

    Serial.begin(115200);

    Serial.println("Program start! Calibrate actuator...");

    MAYaw.initialize();
    MAPitch.initialize();
}



float calc_dt()
{
    cur_millis = millis();
    float ret = (cur_millis - prev_millis)/1000.0f;
    prev_millis = cur_millis;
    if(ret < 0.00001f)
        ret = 0.001f;
    return ret;
}

void processCommand(char * buf)
{
    float * toset = 0;
    switch(buf[0])
    {
    case 'p':
        toset = &MAYaw.PID.coefficients.p;
        break;
    case 'P':
        toset = &MAPitch.PID.coefficients.p;
        break;
    case 'i':
        toset = &MAYaw.PID.coefficients.i;
        break;
    case 'I':
        toset = &MAPitch.PID.coefficients.i;
        break;
    case 'd':
        toset = &MAYaw.PID.coefficients.d;
        break;
    case 'D':
        toset = &MAPitch.PID.coefficients.d;
        break;
    case 'h':
        toset = &phas1;
        break;
    case 'H':
        toset = &phas2;
        break;
    case 'f':
        toset = &freq1;
        break;
    case 'F':
        toset = &freq2;
        break;
    case 'g':
        Serial.println(dt, 10);
        return;
        break;
    case 'a':
        toset = &ampl1;
        break;
    case 'A':
        toset = &ampl2;
        break;
    case 'l':
        toset = &laserf;
        break;
    default:
        return;
    }

    Serial.write(buf[0]);

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

    float sint1 = ampl1 * sin(t*freq1*TWO_PI);
    float sint2 = ampl2 * sin(t*freq2*TWO_PI);
    float lasersint = sin(t*laserf*TWO_PI);

    if(laserf > 0)
    {
        analogWrite(9, ((int)(255 * (lasersint/2.0f + 0.5f))) & 0xFF);
    }
    else
    {
        digitalWrite(9, (laserf < -1.0f)?HIGH:LOW);
    }


    MAYaw.setPos(sint1);
    MAPitch.setPos(sint2);

    MAYaw.update(dt);
    MAPitch.update(dt);

    if(Serial.available())
    {
        ibuffer[ibufferIndex] = Serial.read();
        if(ibuffer[ibufferIndex++] == '\n')
        {
            processCommand(ibuffer);
            ibufferIndex = 0;
        }

        if(ibufferIndex == 32)
        {
            ibufferIndex = 0;
            Serial.println("Input too long!");
        }
    }
}

