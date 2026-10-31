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
#ifndef UTILITY_VISION_FOURCC
#define UTILITY_VISION_FOURCC

#include <cstdint>
#include <string>

namespace utility::vision {

    // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays)
    inline constexpr uint32_t fourcc(const char (&c)[5]) {
        return c[3] << 24 | c[2] << 16 | c[1] << 8 | c[0];
    }

    inline std::string fourcc(const uint32_t& v) {
        return {static_cast<char>((v >> 0) & 0xFF),
                static_cast<char>((v >> 8) & 0xFF),
                static_cast<char>((v >> 16) & 0xFF),
                static_cast<char>((v >> 24) & 0xFF)};
    }

    /// @brief Map a ROS sensor_msgs/Image encoding string to a FOURCC code, or 0 if unsupported.
    inline uint32_t ros_encoding_to_fourcc(const std::string& enc) {
        // clang-format off
        if (enc == "rgb8")         return fourcc("RGB3");
        if (enc == "bgr8")         return fourcc("BGR3");
        if (enc == "mono8")        return fourcc("GREY");
        if (enc == "bayer_bggr8")  return fourcc("BGGR");
        if (enc == "bayer_grbg8")  return fourcc("GRBG");
        if (enc == "bayer_rggb8")  return fourcc("RGGB");
        if (enc == "bayer_gbrg8")  return fourcc("GBRG");
        if (enc == "nv12")         return fourcc("NV12");
        // clang-format on
        return 0;
    }

}  // namespace utility::vision

#endif  // UTILITY_VISION_FOURCC
