#include "DarwinJointMapping.h"

DarwinJointMapping::DarwinJointMapping()
{

    float temp_servo_Offsets[] = 		{	0.0, 	0.0, \
                                                    //JointID::ID_L_SHOULDER_ROLL, JointID::ID_L_SHOULDER_PITCH, JointID::ID_L_ELBOW,
                                                    -0.7853981, 1.5707963, -1.5707963, \
                                                    //JointID::ID_R_SHOULDER_ROLL, JointID::ID_R_SHOULDER_PITCH, JointID::ID_R_ELBOW,
                                                    0.7853981, -1.5707963, 1.5707963, \
                                                    //JointID::ID_L_HIP_ROLL, JointID::ID_L_HIP_PITCH, JointID::ID_L_HIP_YAW,
                                                    0.0,	0.0,	0.0,	\
                                                    //JointID::ID_L_KNEE, JointID::ID_L_ANKLE_ROLL, JointID::ID_L_ANKLE_PITCH,
                                                    0.0,	0.0,	0.0,	\
                                                    //JointID::ID_R_HIP_ROLL, JointID::ID_R_HIP_PITCH, JointID::ID_R_HIP_YAW,
                                                    0.0,	0.0,	0.0,	\
                                                    //JointID::ID_R_KNEE, JointID::ID_R_ANKLE_ROLL, JointID::ID_R_ANKLE_PITCH};
                                                    0.0,	0.0,	0.0};
    m_servo_offsets = std::vector<float>(temp_servo_Offsets, temp_servo_Offsets + sizeof(temp_servo_Offsets)/sizeof(*temp_servo_Offsets));
    return;
}

float DarwinJointMapping::convertJointPosition(unsigned int id, float position)
{
    float value;
    switch(id)
    {
        // Values that require negation.
        case 0:     // Head Pitch
        case 2:     // Left Shoulder Roll
        case 5:     // Right Shoulder Roll
        case 6:     // Right Shoulder Pitch
        case 7:     // Right Elbow Pitch
        case 8:     // Left Hip Roll
        case 9:     // Left Hip Pitch
        case 10:    // Left Hip Yaw
        case 11:    // Right Knee Pitch
        case 14:    // Right Hip Roll
        case 16:    // Right Ankle Pitch
        case 19:    // Right Ankle Pitch
           value = -position;
           break;
        // All other values do not require negation
        default:
            value = position;
    }
    return value;
}

std::vector<float> DarwinJointMapping::convertJointPositionValues(std::vector<float> joints)
{
    for (unsigned int id = 0; id < joints.size(); ++id)
    {
        joints[id] = convertJointPosition(id, joints[id]);
    }
    return joints;
}
