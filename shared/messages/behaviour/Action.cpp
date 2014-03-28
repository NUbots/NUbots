#include "Action.h"

namespace messages {
    namespace behaviour {
        
        using messages::input::ServoID;
        
        std::set<ServoID> servosForLimb(const LimbID& limb) {
            switch(limb) {
                case LimbID::HEAD:
                    return {
                        ServoID::HEAD_PITCH,
                        ServoID::HEAD_YAW
                    };
                    
                case LimbID::LEFT_LEG:
                    return {
                        ServoID::L_ANKLE_PITCH,
                        ServoID::L_ANKLE_ROLL,
                        ServoID::L_HIP_PITCH,
                        ServoID::L_HIP_ROLL,
                        ServoID::L_HIP_YAW,
                        ServoID::L_KNEE
                    };
                    
                case LimbID::RIGHT_LEG:
                    return {
                        ServoID::R_ANKLE_PITCH,
                        ServoID::R_ANKLE_ROLL,
                        ServoID::R_HIP_PITCH,
                        ServoID::R_HIP_ROLL,
                        ServoID::R_HIP_YAW,
                        ServoID::R_KNEE
                    };
                    
                case LimbID::LEFT_ARM:
                    return {
                        ServoID::L_SHOULDER_PITCH,
                        ServoID::L_SHOULDER_ROLL,
                        ServoID::L_ELBOW
                    };
                    
                case LimbID::RIGHT_ARM:
                    return {
                        ServoID::R_SHOULDER_PITCH,
                        ServoID::R_SHOULDER_ROLL,
                        ServoID::R_ELBOW
                    };
                    
                default: {
                    return std::set<ServoID>{};
                }
            }
        }
        
        LimbID limbForServo(const ServoID& servo) {
            switch(servo) {
                case ServoID::HEAD_PITCH:
                case ServoID::HEAD_YAW:
                    return LimbID::HEAD;
                    
                case ServoID::L_ANKLE_PITCH:
                case ServoID::L_ANKLE_ROLL:
                case ServoID::L_HIP_PITCH:
                case ServoID::L_HIP_ROLL:
                case ServoID::L_HIP_YAW:
                case ServoID::L_KNEE:
                    return LimbID::LEFT_LEG;
                    
                case ServoID::R_ANKLE_PITCH:
                case ServoID::R_ANKLE_ROLL:
                case ServoID::R_HIP_PITCH:
                case ServoID::R_HIP_ROLL:
                case ServoID::R_HIP_YAW:
                case ServoID::R_KNEE:
                    return LimbID::RIGHT_LEG;
                    
                case ServoID::L_SHOULDER_PITCH:
                case ServoID::L_SHOULDER_ROLL:
                case ServoID::L_ELBOW:
                    return LimbID::LEFT_ARM;
                    
                case ServoID::R_SHOULDER_PITCH:
                case ServoID::R_SHOULDER_ROLL:
                case ServoID::R_ELBOW:
                    return LimbID::RIGHT_ARM;
            }
        }
    }
}

