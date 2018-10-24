#ifndef MODULE_PLATFORM_NUGUS_NUGUS_H
#define MODULE_PLATFORM_NUGUS_NUGUS_H

#include "DynamixelServo.h"
#include "OpenCR.h"

namespace module {
namespace platform {
    namespace nugus {

        struct MX64 : DynamixelServo {
            uint8_t ID;
            MX64(uint8_t ID) : ID(ID) {}
        };

        struct MX106 : DynamixelServo {
            uint8_t ID;
            MX106(uint8_t ID) : ID(ID) {}
        };

        class NUgus {
        public:
            enum ID {
                OPENCR           = 200,
                R_SHOULDER_PITCH = 1,
                L_SHOULDER_PITCH = 2,
                R_SHOULDER_ROLL  = 3,
                L_SHOULDER_ROLL  = 4,
                R_ELBOW          = 5,
                L_ELBOW          = 6,
                R_HIP_YAW        = 7,
                L_HIP_YAW        = 8,
                R_HIP_ROLL       = 9,
                L_HIP_ROLL       = 10,
                R_HIP_PITCH      = 11,
                L_HIP_PITCH      = 12,
                R_KNEE           = 13,
                L_KNEE           = 14,
                R_ANKLE_PITCH    = 15,
                L_ANKLE_PITCH    = 16,
                R_ANKLE_ROLL     = 17,
                L_ANKLE_ROLL     = 18,
                HEAD_YAW         = 19,
                HEAD_PITCH       = 20,
                BROADCAST        = 254
            };

            static OpenCR OPENCR         = OpenCR(ID::OPENCR);
            static MX64 R_SHOULDER_PITCH = MX64(ID::R_SHOULDER_PITCH);
            static MX64 L_SHOULDER_PITCH = MX64(ID::L_SHOULDER_PITCH);
            static MX64 R_SHOULDER_ROLL  = MX64(ID::R_SHOULDER_ROLL);
            static MX64 L_SHOULDER_ROLL  = MX64(ID::L_SHOULDER_ROLL);
            static MX64 R_ELBOW          = MX64(ID::R_ELBOW);
            static MX64 L_ELBOW          = MX64(ID::L_ELBOW);
            static MX106 R_HIP_YAW       = MX106(ID::R_HIP_YAW);
            static MX106 L_HIP_YAW       = MX106(ID::L_HIP_YAW);
            static MX106 R_HIP_ROLL      = MX106(ID::R_HIP_ROLL);
            static MX106 L_HIP_ROLL      = MX106(ID::L_HIP_ROLL);
            static MX106 R_HIP_PITCH     = MX106(ID::R_HIP_PITCH);
            static MX106 L_HIP_PITCH     = MX106(ID::L_HIP_PITCH);
            static MX106 R_KNEE          = MX106(ID::R_KNEE);
            static MX106 L_KNEE          = MX106(ID::L_KNEE);
            static MX106 R_ANKLE_PITCH   = MX106(ID::R_ANKLE_PITCH);
            static MX106 L_ANKLE_PITCH   = MX106(ID::L_ANKLE_PITCH);
            static MX106 R_ANKLE_ROLL    = MX106(ID::R_ANKLE_ROLL);
            static MX106 L_ANKLE_ROLL    = MX106(ID::L_ANKLE_ROLL);
            static MX64 HEAD_YAW         = MX64(ID::HEAD_YAW);
            static MX64 HEAD_PITCH       = MX64(ID::HEAD_PITCH);

            static constexpr DynamixelDevice& operator[](const ID& id) {
                switch (id) {
                    case ID::OPENCR: return OPENCR;
                    case ID::R_SHOULDER_PITCH: return R_SHOULDER_PITCH;
                    case ID::L_SHOULDER_PITCH: return L_SHOULDER_PITCH;
                    case ID::R_SHOULDER_ROLL: return R_SHOULDER_ROLL;
                    case ID::L_SHOULDER_ROLL: return L_SHOULDER_ROLL;
                    case ID::R_ELBOW: return R_ELBOW;
                    case ID::L_ELBOW: return L_ELBOW;
                    case ID::R_HIP_YAW: return R_HIP_YAW;
                    case ID::L_HIP_YAW: return L_HIP_YAW;
                    case ID::R_HIP_ROLL: return R_HIP_ROLL;
                    case ID::L_HIP_ROLL: return L_HIP_ROLL;
                    case ID::R_HIP_PITCH: return R_HIP_PITCH;
                    case ID::L_HIP_PITCH: return L_HIP_PITCH;
                    case ID::R_KNEE: return R_KNEE;
                    case ID::L_KNEE: return L_KNEE;
                    case ID::R_ANKLE_PITCH: return R_ANKLE_PITCH;
                    case ID::L_ANKLE_PITCH: return L_ANKLE_PITCH;
                    case ID::R_ANKLE_ROLL: return R_ANKLE_ROLL;
                    case ID::L_ANKLE_ROLL: return L_ANKLE_ROLL;
                    case ID::HEAD_YAW: return HEAD_YAW;
                    case ID::HEAD_PITCH: return HEAD_PITCH;
                    default: throw std::runtime_error("Unknown device id");
                }
            }
        };

#pragma pack(push, 1)  // Here we disable the OS putting in padding bytes so we can raw memcpy into this data
        struct DynamixelServoWriteData {
            uint8_t TORQUE_ENABLE;
            uint16_t VELOCITY_I_GAIN;
            uint16_t VELOCITY_P_GAIN;
            uint16_t VELOCITY_D_GAIN;
            uint16_t POSITION_I_GAIN;
            uint16_t POSITION_P_GAIN;
            uint16_t FEEDFORWARD_1ST_GAIN;
            uint16_t FEEDFORWARD_2ND_GAIN;
            uint16_t GOAL_PWM;
            uint16_t GOAL_CURRENT;
            uint32_t GOAL_VELOCITY;
            uint32_t PROFILE_ACCELERATION;
            uint32_t PROFILE_VELOCITY;
            uint32_t GOAL_POSITION;
        };

        struct DynamixelServoReadData {
            uint8_t HARDWARE_ERROR_STATUS;
            uint16_t PRESENT_PWM;
            uint16_t PRESENT_CURRENT;
            uint32_t PRESENT_VELOCITY;
            uint32_t PRESENT_POSITION;
            uint16_t PRESENT_VOLTAGE;
            uint8_t PRESENT_TEMPERATURE;
        };

        struct OpenCRWriteData {
            uint8_t LED;
            uint16_t RGB_LED;
            uint16_t Buzzer;
        };

        struct OpenCRReadData {
            uint8_t LED;
            uint16_t RGB_LED;
            uint16_t Buzzer;
            uint8_t button;
            uint8_t voltage;
            uint16_t gyro_z;
            uint16_t gyro_y;
            uint16_t gyro_x;
            uint16_t acc_x;
            uint16_t acc_y;
            uint16_t acc_z;
        };
#pragma pack(pop)  // Stop bitpacking our results

    }  // namespace nugus
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_NUGUS_NUGUS_H
