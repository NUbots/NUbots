#include "NUgus.hpp"

namespace module::platform::OpenCR {

    NUgus::NUgus()
        : OPENCR(uint8_t(ID::OPENCR))
        , R_SHOULDER_PITCH(uint8_t(ID::R_SHOULDER_PITCH))
        , L_SHOULDER_PITCH(uint8_t(ID::L_SHOULDER_PITCH))
        , R_SHOULDER_ROLL(uint8_t(ID::R_SHOULDER_ROLL))
        , L_SHOULDER_ROLL(uint8_t(ID::L_SHOULDER_ROLL))
        , R_ELBOW(uint8_t(ID::R_ELBOW))
        , L_ELBOW(uint8_t(ID::L_ELBOW))
        , R_HIP_YAW(uint8_t(ID::R_HIP_YAW))
        , L_HIP_YAW(uint8_t(ID::L_HIP_YAW))
        , R_HIP_ROLL(uint8_t(ID::R_HIP_ROLL))
        , L_HIP_ROLL(uint8_t(ID::L_HIP_ROLL))
        , R_HIP_PITCH(uint8_t(ID::R_HIP_PITCH))
        , L_HIP_PITCH(uint8_t(ID::L_HIP_PITCH))
        , R_KNEE(uint8_t(ID::R_KNEE))
        , L_KNEE(uint8_t(ID::L_KNEE))
        , R_ANKLE_PITCH(uint8_t(ID::R_ANKLE_PITCH))
        , L_ANKLE_PITCH(uint8_t(ID::L_ANKLE_PITCH))
        , R_ANKLE_ROLL(uint8_t(ID::R_ANKLE_ROLL))
        , L_ANKLE_ROLL(uint8_t(ID::L_ANKLE_ROLL))
        , HEAD_YAW(uint8_t(ID::HEAD_YAW))
        , HEAD_PITCH(uint8_t(ID::HEAD_PITCH)) {}

}  // namespace module::platform::OpenCR
