#ifndef DARWIN_H
#define DARWIN_H

#include <thread>
#include <string>
#include "CM730.h"
#include "MX28.h"
#include "FSR.h"
#include "messages/DarwinRawSensors.h"

namespace Darwin {
class Darwin {
private:
    UART m_uart;
    CM730 m_cm730;
    MX28 m_rShoulderPitch;
    MX28 m_lShoulderPitch;
    MX28 m_rShoulderRoll;
    MX28 m_lShoulderRoll;
    MX28 m_rElbow;
    MX28 m_lElbow;
    MX28 m_rHipYaw;
    MX28 m_lHipYaw;
    MX28 m_rHipRoll;
    MX28 m_lHipRoll;
    MX28 m_rHipPitch;
    MX28 m_lHipPitch;
    MX28 m_rKnee;
    MX28 m_lKnee;
    MX28 m_rAnklePitch;
    MX28 m_lAnklePitch;
    MX28 m_rAnkleRoll;
    MX28 m_lAnkleRoll;
    MX28 m_headPan;
    MX28 m_headTilt;
    FSR m_rFSR;
    FSR m_lFSR;
    
public:
    Darwin(const char* name);
    
    void selfTest();
    void readAll();
};
}

#endif