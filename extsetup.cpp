#include "extsetup.h"
#include "PIDControl.h"
#include <Wire.h>
#include <stdlib.h>
#include "Smoother.h"
#include "MirrorAxis.h"

float calc_dt();

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

int accum = 0;

void extsetup()
{
    Wire.begin();

    Serial.begin(115200);

    Serial.println("Program start! Calibrate actuator...");


}



float calc_dt()
{
    cur_millis = millis();
    float ret = (cur_millis - prev_millis)/1000.0f;
    prev_millis = cur_millis;
    return ret;
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

