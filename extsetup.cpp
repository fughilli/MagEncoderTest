/*
 * extsetup.cpp
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

#include "extsetup.h"
#include "PIDControl.h"
#include <Wire.h>
#include <stdlib.h>
#include "Smoother.h"
#include "MirrorAxis.h"

// GLOBAL VARIABLES
float dt = 0;

float globalMult = 1.0f;

#define COMM_IBUFFER_SIZE 32
char ibuffer[COMM_IBUFFER_SIZE];
uint8_t ibufferIndex = 0;

int prev_millis = 0, cur_millis = 0;

typedef void(*demoUpdateFunction_t)(float, float);
typedef void(*demoInitFunction_t)(void);

demoUpdateFunction_t demoUpdateFunction = NULL;

// FUNCTION DECLARATIONS
float calc_dt();

void circleUpdateFunction(float t, float dt);
void boxUpdateFunction(float t, float dt);
void line1UpdateFunction(float t, float dt);
void line2UpdateFunction(float t, float dt);
void snakeUpdateFunction(float t, float dt);
void bowtievUpdateFunction(float t, float dt);
void bowtiehUpdateFunction(float t, float dt);
void snake2UpdateFunction(float t, float dt);
void tone1sUpdateFunction(float t, float dt);

void circleInitFunction();
void boxInitFunction();
void line1InitFunction();
void line2InitFunction();
void snakeInitFunction();
void bowtievInitFunction();
void bowtiehInitFunction();
void idleInitFunction();
void snake2InitFunction();
void tone1sInitFunction();

// GLOBAL CLASS INSTANCES
MirrorAxis MAYaw(0x40, 6, 2, 5), MAPitch(0x41, 7, 4, 10);

// DEMO LOOKUP TABLES
#define NUM_COMMANDS 10
#define MAX_COMMAND_LEN 8

const char commandStrings[NUM_COMMANDS][MAX_COMMAND_LEN] =
{
    "idle",
    "circle",
    "box",
    "line1",
    "line2",
    "snake",
    "bowtiev",
    "bowtieh",
    "snake2",
    "tone1s"
};

const demoUpdateFunction_t commandUpdateFuncs[NUM_COMMANDS] =
{
    NULL,
    circleUpdateFunction,
    boxUpdateFunction,
    line1UpdateFunction,
    line2UpdateFunction,
    snakeUpdateFunction,
    bowtievUpdateFunction,
    bowtiehUpdateFunction,
    snake2UpdateFunction,
    tone1sUpdateFunction,
};

const demoInitFunction_t commandInitFuncs[NUM_COMMANDS] =
{
    idleInitFunction,
    circleInitFunction,
    boxInitFunction,
    line1InitFunction,
    line2InitFunction,
    snakeInitFunction,
    bowtievInitFunction,
    bowtiehInitFunction,
    snake2InitFunction,
    tone1sInitFunction
};

void extsetup()
{
    Wire.begin();               // Initialize I2C bus

    Serial.begin(115200);       // Initialize UART

    Serial.println("Program start! Calibrate actuator...");

    pinMode(9, OUTPUT);         // Set the laser control pin to OUTPUT

    idleInitFunction();         // Start in IDLE state

    MAYaw.initialize();         // Initialize Yaw/Pitch axes
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
    for(int i = 0; i < NUM_COMMANDS; i++)
    {
        if(strncmp(buf, commandStrings[i], COMM_IBUFFER_SIZE) == 0)
        {
            demoUpdateFunction = commandUpdateFuncs[i];
            commandInitFuncs[i]();

            Serial.println(commandStrings[i]);
            return;
        }
    }

    if(strncmp(buf, "mult", 4) == 0)
    {
        globalMult = atof(buf + 4);
        Serial.print("new mult: ");
        Serial.println(globalMult);
        return;
    }

    Serial.println("unrecognized command");
}

float t = 0;

void extloop()
{
    dt = calc_dt();
    t += dt;

    if(t > 1000.0f)
    {
        t = 0.0f;
    }

    if(demoUpdateFunction != NULL)
    {
        demoUpdateFunction(globalMult * t, dt);
    }

    MAYaw.update(dt);
    MAPitch.update(dt);

    if(Serial.available())
    {
        ibuffer[ibufferIndex] = Serial.read();
        if(ibuffer[ibufferIndex++] == '\n')
        {
            ibuffer[ibufferIndex - 1] = '\0';
            processCommand(ibuffer);
            ibufferIndex = 0;
        }

        if(ibufferIndex == COMM_IBUFFER_SIZE)
        {
            ibufferIndex = 0;
            Serial.println("Input too long!");
        }
    }
}

void setLaser(float lp)
{
    if(lp >= 1)
    {
        digitalWrite(9, HIGH);
    }
    else if(lp > 0)
    {
        analogWrite(9, (255 * lp));
    }
    else
    {
        digitalWrite(9, LOW);
    }
}

void circleInitFunction()
{
    setLaser(1);
}

void circleUpdateFunction(float t, float dt)
{
    float sint1 = 0.6 * sin((t*10 + 0.25)*TWO_PI);
    float sint2 = 0.3 * sin((t*10 + 0)*TWO_PI);

    MAYaw.setPos(sint1);
    MAPitch.setPos(sint2);

    //float lasersint = (((int)(t*laserf))%2)?1.0f:-1.0f;//sin(t*laserf*TWO_PI);
}

void boxUpdateFunction(float t, float dt)
{
    t *= 4.0f;

    float t1 = fmod(t, 1.0f)*2.0f - 1.0f;

    switch(((int)t) % 4)
    {
    case 0:
        MAYaw.setPos(0.6f*t1);
        break;
    case 1:
        MAPitch.setPos(0.3f*t1);
        break;
    case 2:
        MAYaw.setPos(-0.6f*t1);
        break;
    case 3:
        MAPitch.setPos(-0.3f*t1);
        break;
    }
}

void line1UpdateFunction(float t, float dt)
{
    float sint1 = 0.6 * sin((t*5 + 0)*TWO_PI);
    float sint2 = 0.3 * sin((t*5 + 0)*TWO_PI);

    MAYaw.setPos(sint1);
    MAPitch.setPos(sint2);
}

void line2UpdateFunction(float t, float dt)
{
    float sint1 = 0.6 * sin((t*5 - 0.5)*TWO_PI);
    float sint2 = 0.3 * sin((t*5 + 0)*TWO_PI);

    MAYaw.setPos(sint1);
    MAPitch.setPos(sint2);
}

void snakeUpdateFunction(float t, float dt)
{
    circleUpdateFunction(t, dt);

    float lasersint = (((int)(t*20.5f))%2)?1.0f:0.0f;

    setLaser(lasersint);
}

void snake2UpdateFunction(float t, float dt)
{
    circleUpdateFunction(t, dt);

    float lasersint = (sin(t*20.5*TWO_PI)+1.0f)/2.0f;

    setLaser(lasersint);
}

void bowtievUpdateFunction(float t, float dt)
{
    float sint1 = 0.6 * sin((t*10)*TWO_PI);
    float sint2 = 0.3 * sin((t*5)*TWO_PI);

    MAYaw.setPos(sint1);
    MAPitch.setPos(sint2);
}

void bowtiehUpdateFunction(float t, float dt)
{
    float sint1 = 0.6 * sin((t*5)*TWO_PI);
    float sint2 = 0.3 * sin((t*10)*TWO_PI);

    MAYaw.setPos(sint1);
    MAPitch.setPos(sint2);
}

void snake2InitFunction()
{

}

void idleInitFunction()
{
    Serial.println("setLaser -1");
    setLaser(-1);

    MAYaw.setPos(0);
    MAPitch.setPos(0);
}

void boxInitFunction()
{
    setLaser(1);
}

void line1InitFunction()
{
    setLaser(1);
}

void line2InitFunction()
{
    setLaser(1);
}

void snakeInitFunction()
{

}

void bowtievInitFunction()
{
    setLaser(1);
}

void bowtiehInitFunction()
{
    setLaser(1);
}

float toneCount = 0;

void tone1sInitFunction()
{
    tone(8, 1000 * globalMult);
    toneCount = 0;
}

void tone1sUpdateFunction(float t, float dt)
{
    if(toneCount >= 0)
        toneCount += dt;
    if(toneCount > 2)
    {
        toneCount = -1;
        noTone(8);
    }
}
