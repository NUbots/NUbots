#include "Darwin.h"

Darwin::Darwin::Darwin(const char* name) :
m_uart(name),
m_cm730(m_uart, 200),
m_rShoulderPitch(m_uart, 1),
m_lShoulderPitch(m_uart, 2),
m_rShoulderRoll(m_uart, 3),
m_lShoulderRoll(m_uart, 4),
m_rElbow(m_uart, 5),
m_lElbow(m_uart, 6),
m_rHipYaw(m_uart, 7),
m_lHipYaw(m_uart, 8),
m_rHipRoll(m_uart, 9),
m_lHipRoll(m_uart, 10),
m_rHipPitch(m_uart, 11),
m_lHipPitch(m_uart, 12),
m_rKnee(m_uart, 13),
m_lKnee(m_uart, 14),
m_rAnklePitch(m_uart, 15),
m_lAnklePitch(m_uart, 16),
m_rAnkleRoll(m_uart, 17),
m_lAnkleRoll(m_uart, 18),
m_headPan(m_uart, 19),
m_headTilt(m_uart, 20),
m_rFSR(m_uart, 111),
m_lFSR(m_uart, 112) {
    
    // Turn on the dynamixel power
    m_cm730.turnOnDynamixel();
    
    // Wait about 300ms for the dynamixels to start up
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
    // Run a self test (test which sensors are working and which are not)
    selfTest();
}

void Darwin::Darwin::selfTest() {
    
    // Ping all our motors
    for (int i = 0; i < 20; ++i) {
        (&m_rShoulderPitch)[i].ping();
    }
    
    // Ping our two FSRs
    m_lFSR.ping();
    m_rFSR.ping();
}

void Darwin::Darwin::readAll() {
    
    // Here we construct our bulk read packet
    
    // Do the header stuff
}