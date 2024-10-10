/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_INPUT_CAMERA_DESCRIPTION_TO_FOURCC_HPP
#define MODULE_INPUT_CAMERA_DESCRIPTION_TO_FOURCC_HPP

#include <string>

#include "utility/vision/fourcc.hpp"

namespace module::input {

    inline uint32_t description_to_fourcc(const std::string& code) {
        using namespace utility::vision;

        // clang-format off
        if (code == "Mono8")               { return fourcc("GREY"); }
        if (code == "Mono12Packed")        { return fourcc("Y12 "); }
        if (code == "Mono12p")             { return fourcc("Y12 "); }
        if (code == "Mono16")              { return fourcc("Y16 "); }
        if (code == "BayerGR8")            { return fourcc("GRBG"); }
        if (code == "BayerRG8")            { return fourcc("RGGB"); }
        if (code == "BayerGB8")            { return fourcc("GBRG"); }
        if (code == "BayerBG8")            { return fourcc("BGGR"); }
        if (code == "BayerRGPolarized8")   { return fourcc("PRG8"); }
        if (code == "BayerGRPolarized8")   { return fourcc("PGR8"); }
        if (code == "BayerGBPolarized8")   { return fourcc("PGB8"); }
        if (code == "BayerBGPolarized8")   { return fourcc("PBG8"); }
        if (code == "BayerGR12p")          { return fourcc("GR12"); }
        if (code == "BayerRG12p")          { return fourcc("RG12"); }
        if (code == "BayerGB12p")          { return fourcc("GB12"); }
        if (code == "BayerBG12p")          { return fourcc("BG12"); }
        if (code == "BayerGR12Packed")     { return fourcc("GR12"); }
        if (code == "BayerRG12Packed")     { return fourcc("RG12"); }
        if (code == "BayerGB12Packed")     { return fourcc("GB12"); }
        if (code == "BayerBG12Packed")     { return fourcc("BG12"); }
        if (code == "BayerGR16")           { return fourcc("GR16"); }
        if (code == "BayerRG16")           { return fourcc("RG16"); }
        if (code == "BayerGB16")           { return fourcc("GB16"); }
        if (code == "BayerBG16")           { return fourcc("BG16"); }
        if (code == "YCbCr411_8_CbYYCrYY") { return fourcc("Y411"); }
        if (code == "YCbCr422_8_CbYCrY")   { return fourcc("UYVY"); }
        if (code == "YCbCr8_CbYCr")        { return fourcc("YM24"); }
        if (code == "YUYV")                { return fourcc("YUYV"); }
        if (code == "RGB8")                { return fourcc("RGB3"); }
        // clang-format on

        throw std::runtime_error("Could not find a fourcc for the given description");
    }
}  // namespace module::input

#endif  // MODULE_INPUT_CAMERA_DESCRIPTION_TO_FOURCC_HPP
