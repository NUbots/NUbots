#ifndef DARWIN_H
#define DARWIN_H

#include <thread>
#include <string>
#include <cstring>

#include "CM730.h"
#include "MX28.h"
#include "FSR.h"

#include "DarwinRawSensors.h"

namespace Darwin {
    
    namespace ID {
        enum {
            CM730 = 200,
            R_SHOULDER_PITCH = 1,
            L_SHOULDER_PITCH = 2,
            R_SHOULDER_ROLL = 3,
            L_SHOULDER_ROLL = 4,
            R_ELBOW = 5,
            L_ELBOW = 6,
            R_HIP_YAW = 7,
            L_HIP_YAW = 8,
            R_HIP_ROLL = 9,
            L_HIP_ROLL = 10,
            R_HIP_PITCH = 11,
            L_HIP_PITCH = 12,
            R_KNEE = 13,
            L_KNEE = 14,
            R_ANKLE_PITCH = 15,
            L_ANKLE_PITCH = 16,
            R_ANKLE_ROLL = 17,
            L_ANKLE_ROLL = 18,
            HEAD_PAN = 19,
            HEAD_TILT = 20,
            R_FSR = 111,
            L_FSR = 112
        };
    }
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
    
    std::vector<uint8_t> m_bulkReadCommand;
    
    void buildBulkReadPacket();
    
public:
    Darwin(const char* name);
    
    void selfTest();
    BulkReadResults bulkRead();
    void writeMotors();
};
}

#endif