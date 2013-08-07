#include "DarwinSensors.h"

Messages::DarwinSensors::Servo& Messages::DarwinSensors::Servos::operator[](int index) {

    switch (index) {
        case Servo::ID::HEAD_PAN:           return headPan;
        case Servo::ID::HEAD_TILT:          return headTilt;
        case Servo::ID::R_SHOULDER_PITCH:   return rShoulderPitch;
        case Servo::ID::L_SHOULDER_PITCH:   return lShoulderPitch;
        case Servo::ID::R_SHOULDER_ROLL:    return rShoulderRoll;
        case Servo::ID::L_SHOULDER_ROLL:    return lShoulderRoll;
        case Servo::ID::R_ELBOW:            return rElbow;
        case Servo::ID::L_ELBOW:            return lElbow;
        case Servo::ID::R_HIP_YAW:          return rHipYaw;
        case Servo::ID::L_HIP_YAW:          return lHipYaw;
        case Servo::ID::R_HIP_ROLL:         return rHipRoll;
        case Servo::ID::L_HIP_ROLL:         return lHipRoll;
        case Servo::ID::R_HIP_PITCH:        return rHipPitch;
        case Servo::ID::L_HIP_PITCH:        return lHipPitch;
        case Servo::ID::R_KNEE:             return rKnee;
        case Servo::ID::L_KNEE:             return lKnee;
        case Servo::ID::R_ANKLE_PITCH:      return rAnklePitch;
        case Servo::ID::L_ANKLE_PITCH:      return lAnklePitch;
        case Servo::ID::R_ANKLE_ROLL:       return rAnkleRoll;
        case Servo::ID::L_ANKLE_ROLL:       return lAnkleRoll;
    }

    throw std::runtime_error("Out of bounds");
}