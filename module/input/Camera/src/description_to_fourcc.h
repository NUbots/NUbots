#ifndef MODULE_INPUT_CAMERA_DESCRIPTION_TO_FOURCC_H
#define MODULE_INPUT_CAMERA_DESCRIPTION_TO_FOURCC_H

#include <string>

#include "utility/vision/fourcc.h"

namespace module {
namespace input {

    inline uint32_t description_to_fourcc(const std::string& code) {
        using namespace utility::vision;

        // clang-format off
             if (code == "Mono8")               return fourcc("GREY");
        else if (code == "Mono12Packed")        return fourcc("Y12 ");
        else if (code == "Mono12p")             return fourcc("Y12 ");
        else if (code == "Mono16")              return fourcc("Y16 ");
        else if (code == "BayerGR8")            return fourcc("GRBG");
        else if (code == "BayerRG8")            return fourcc("RGGB");
        else if (code == "BayerGB8")            return fourcc("GBRG");
        else if (code == "BayerBG8")            return fourcc("BGGR");
        else if (code == "BayerRGPolarized8")   return fourcc("PRG8");
        else if (code == "BayerGRPolarized8")   return fourcc("PGR8");
        else if (code == "BayerGBPolarized8")   return fourcc("PGB8");
        else if (code == "BayerBGPolarized8")   return fourcc("PBG8");
        else if (code == "BayerGR12p")          return fourcc("GR12");
        else if (code == "BayerRG12p")          return fourcc("RG12");
        else if (code == "BayerGB12p")          return fourcc("GB12");
        else if (code == "BayerBG12p")          return fourcc("BG12");
        else if (code == "BayerGR12Packed")     return fourcc("GR12");
        else if (code == "BayerRG12Packed")     return fourcc("RG12");
        else if (code == "BayerGB12Packed")     return fourcc("GB12");
        else if (code == "BayerBG12Packed")     return fourcc("BG12");
        else if (code == "BayerGR16")           return fourcc("GR16");
        else if (code == "BayerRG16")           return fourcc("RG16");
        else if (code == "BayerGB16")           return fourcc("GB16");
        else if (code == "BayerBG16")           return fourcc("BG16");
        else if (code == "YCbCr411_8_CbYYCrYY") return fourcc("Y411");
        else if (code == "YCbCr422_8_CbYCrY")   return fourcc("UYVY");
        else if (code == "YCbCr8_CbYCr")        return fourcc("YM24");
        else if (code == "YUYV")                return fourcc("YUYV");
        else if (code == "RGB8")                return fourcc("RGB3");
        // clang-format on

        throw std::runtime_error("Could not find a fourcc for the given description");
    }

}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_CAMERA_DESCRIPTION_TO_FOURCC_H
