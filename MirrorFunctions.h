#ifndef MIRRORFUNCTIONS_H
#define MIRRORFUNCTIONS_H
#include "PIDControl.h"
#include "Arduino.h"
#include "Wire.h"

class MirrorFunctions
{
    public:
        MirrorFunctions(uint8_t sensorAddress, int drivePin, int leftPin, int rightPin);
        virtual ~MirrorFunctions();

        void setSensorZero(int zeropos);
        int readSensor(char);
        void setactuator(float);
        float getscaledpos();
        float calculate(float x, float target_x, float dt);
        void initactuator();
        PIDController* PID();
        float m_freq;
        float m_ampl;
    protected:
    private:
        PIDController m_pidl;
        int m_actuator_minpos, m_actuator_maxpos;
        //float* m_freq ,m_ampl;
        uint8_t m_sensorAddress;
        int m_drivePin,m_leftPin,m_rightPin;


};

#endif // MIRRORFUNCTIONS_H
