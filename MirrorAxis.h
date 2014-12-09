#ifndef _MIRROR_AXIS_H_
#define _MIRROR_AXIS_H_

#include "PIDControl.h"

#define SENSOR_MAG_ADDR 0xFC
#define SENSOR_ANG_ADDR 0xFF
#define SENSOR_ZERO_ADDR 0x17

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
    float _getPos();
    void setActuatorPower(float x);
    int m_dpin1, m_dpin2, m_drivepin;
    uint8_t m_dev_addr;
    float m_pos, m_setpos;
    int m_actuator_minpos, m_actuator_maxpos;
};

#endif // _MIRROR_AXIS_H_
