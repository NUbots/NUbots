#ifndef DARWINJOINTMAPPING_H
#define DARWINJOINTMAPPING_H

#include <vector>

class DarwinJointMapping
{
public:
    DarwinJointMapping();
    static float convertJointPosition(unsigned int id, float position);
    static std::vector<float> convertJointPositionValues(std::vector<float> joints);
    std::vector<float>m_servo_offsets;
};

#endif // DARWINJOINTMAPPING_H
