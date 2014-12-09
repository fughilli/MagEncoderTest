#ifndef MIRRORFUNCTIONS_H
#define MIRRORFUNCTIONS_H
#include "PIDControl.h"



class MirrorFunctions
{
    public:
        MirrorFunctions();
        virtual ~MirrorFunctions();

        void setSensorZero(int zeropos);
        int readSensor(char);
        void setactuator(float);
        float getscaledpos();
        void calculate(float x, float target_x, float dt);
        void initactuator();

    protected:
    private:
        PIDController m_pidl;
        int m_actuator_minpos, m_actuator_maxpos;
        uint8_t m_sensorAddress;
        int m_drivePin,m_leftPin,m_rightPin;


};

#endif // MIRRORFUNCTIONS_H
