/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */
#ifndef UTILITY_VISION_VISION_H
#define UTILITY_VISION_VISION_H

#include <fmt/format.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

extern "C" {
#include <aravis-0.6/arv.h>
}

#include "message/input/Image.h"

namespace utility {
namespace vision {

    struct Colour {
        enum Value : char {
            // Main classifications
            UNCLASSIFIED = 'u',
            WHITE        = 'w',
            GREEN        = 'g',
            ORANGE       = 'o',
            YELLOW       = 'y',
            CYAN         = 'c',
            MAGENTA      = 'm',

            // Ambiguous Classifications
            WHITE_GREEN = 'f'
        };
        Value value;

        // Constructors
        Colour() : value(Value::UNCLASSIFIED) {}
        Colour(int const& value) : value(static_cast<Value>(value)) {}
        Colour(uint8_t const& value) : value(static_cast<Value>(value)) {}
        Colour(uint32_t const& value) : value(static_cast<Value>(value)) {}
        Colour(char const& value) : value(static_cast<Value>(value)) {}
        Colour(Value const& value) : value(value) {}
        Colour(std::string const& str) : value(Value::UNCLASSIFIED) {
            if (str == "UNCLASSIFIED")
                value = Value::UNCLASSIFIED;
            else if (str == "WHITE")
                value = Value::WHITE;
            else if (str == "GREEN")
                value = Value::GREEN;
            else if (str == "ORANGE")
                value = Value::ORANGE;
            else if (str == "YELLOW")
                value = Value::YELLOW;
            else if (str == "CYAN")
                value = Value::CYAN;
            else if (str == "MAGENTA")
                value = Value::MAGENTA;
            else if (str == "WHITE_GREEN")
                value = Value::WHITE_GREEN;
            else
                throw std::runtime_error("String " + str + " did not match any enum for Colour");
        }


        // Operators
        bool operator<(Colour const& other) const {
            return value < other.value;
        }
        bool operator>(Colour const& other) const {
            return value > other.value;
        }
        bool operator<=(Colour const& other) const {
            return value <= other.value;
        }
        bool operator>=(Colour const& other) const {
            return value >= other.value;
        }
        bool operator==(Colour const& other) const {
            return value == other.value;
        }
        bool operator!=(Colour const& other) const {
            return value != other.value;
        }
        bool operator<(Colour::Value const& other) const {
            return value < other;
        }
        bool operator>(Colour::Value const& other) const {
            return value > other;
        }
        bool operator<=(Colour::Value const& other) const {
            return value <= other;
        }
        bool operator>=(Colour::Value const& other) const {
            return value >= other;
        }
        bool operator==(Colour::Value const& other) const {
            return value == other;
        }
        bool operator!=(Colour::Value const& other) const {
            return value != other;
        }

        // Conversions
        operator Value() const {
            return value;
        }
        operator int() const {
            return value;
        }
        operator uint8_t() const {
            return value;
        }
        operator uint32_t() const {
            return value;
        }

        operator std::string() const {
            switch (value) {
                case Value::UNCLASSIFIED: return "UNCLASSIFIED";
                case Value::WHITE: return "WHITE";
                case Value::GREEN: return "GREEN";
                case Value::ORANGE: return "ORANGE";
                case Value::YELLOW: return "YELLOW";
                case Value::CYAN: return "CYAN";
                case Value::MAGENTA: return "MAGENTA";
                case Value::WHITE_GREEN: return "WHITE_GREEN";
                default:
                    throw std::runtime_error("enum Colour's value is corrupt, unknown value stored"
                                             + std::to_string(static_cast<uint8_t>(value)));
            }
        }

        operator char() const {
            return static_cast<char>(value);
        }
    };

    struct Pixel {
        Pixel() : rgba(0) {}
        Pixel(uint32_t rgba) : rgba(rgba) {}
        Pixel(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : components({r, g, b, a}) {}
        Pixel(uint8_t r, uint8_t g, uint8_t b) : components({r, g, b, 0}) {}
        Pixel(const Pixel& pixel) : rgba(pixel.rgba) {}

        union {
            struct {
                union {
                    uint8_t r;
                    uint8_t y;
                };
                union {
                    uint8_t g;
                    uint8_t u;
                    uint8_t cb;
                };
                union {
                    uint8_t b;
                    uint8_t v;
                    uint8_t cr;
                };

                uint8_t a;
            } components;

            uint32_t rgba;
        };
    };

    enum FOURCC : uint32_t {
        GREY    = 0x59455247,
        Y12     = 0x20323159,
        Y16     = 0x20363159,
        GRBG    = 0x47425247,
        RGGB    = 0x42474752,
        GBRG    = 0x47524247,
        BGGR    = 0x52474742,
        GR12    = 0x32315247,
        RG12    = 0x32314752,
        GB12    = 0x32314247,
        BG12    = 0x32314742,
        GR16    = 0x36315247,
        RG16    = 0x36314752,
        GB16    = 0x36314247,
        BG16    = 0x36314742,
        Y411    = 0x31313459,
        UYVY    = 0x59565955,
        YUYV    = 0x56595559,
        YM24    = 0x34324d59,
        RGB3    = 0x33424752,
        JPEG    = 0x4745504a,
        UNKNOWN = 0
    };

    enum class BayerPixelType {
        R,   // Its red
        GR,  // Green on red row
        GB,  // Green on blue row
        B    // Its blue
    };

    // Implemented from http://www.ipol.im/pub/art/2011/g_mhcd/
    // Malvar-He-Cutler Linear Image Demosaicking
    // Bayer interpolators
    // These are masks, not matrices.
    // Green pixels at both blue and red locations.
    constexpr int8_t BAYER_SCALE = 6;  // 8 = 2^3 .... use bit shift
    // clang-format off
    constexpr int8_t GREEN_AXIAL_ARR[25] = { 0,  0, -8,  0,  0,
                                             0,  0, 16,  0,  0,
                                            -8, 16, 32, 16, -8,
                                             0,  0, 16,  0,  0,
                                             0,  0, -8,  0,  0};
    // clang-format on

    const Eigen::Matrix<int8_t, 5, 5> GREEN_AXIAL =
        Eigen::Map<const Eigen::Matrix<int8_t, 5, 5>>(GREEN_AXIAL_ARR, 5, 5);

    // Red at blue locations and blue at red locations
    // clang-format off
    constexpr int8_t RED_AT_BLUE_ARR[25] = {  0,  0, -12,  0,   0,
                                              0, 16,   0, 16,   0,
                                            -12,  0,  48,  0, -12,
                                              0, 16,   0, 16,   0,
                                              0,  0, -12,  0,   0};
    // clang-format on

    const Eigen::Matrix<int8_t, 5, 5> RED_AT_BLUE =
        Eigen::Map<const Eigen::Matrix<int8_t, 5, 5>>(RED_AT_BLUE_ARR, 5, 5);

    // Red at green locations and blue at green locations, on red rows
    // Red at green locations and blue at green locations, on blue rows are the transpose of this mask.

    // clang-format off
    constexpr int8_t RED_AT_GREEN_ARR[25] = { 0,  0,  4,  0,  0,
                                              0, -8,  0, -8,  0,
                                             -8, 32, 40, 32, -8,
                                              0, -8,  0, -8,  0,
                                              0,  0,  4,  0,  0};
    // clang-format on
    const Eigen::Matrix<int8_t, 5, 5> RED_AT_GREEN =
        Eigen::Map<const Eigen::Matrix<int8_t, 5, 5>>(RED_AT_GREEN_ARR, 5, 5);

    void saveImage(const std::string& file, const message::input::Image& image);

    template <typename T>
    inline void loadImage(const std::string& file, T& image) {
        std::ifstream ifs(file, std::ios::in | std::ios::binary);
        std::string magic_number, width, height, max_val;
        uint8_t bytes_per_pixel;
        bool RGB;
        ifs >> magic_number;

        if (magic_number.compare("P6") == 0) {
            RGB = true;
        }

        else if (magic_number.compare("P5") == 0) {
            RGB = false;
        }

        else {
            throw std::runtime_error("Image has incorrect format.");
            return;
        }

        ifs >> width >> height >> max_val;
        image.dimensions.x() = std::stoi(width);
        image.dimensions.y() = std::stoi(height);
        bytes_per_pixel      = ((std::stoi(max_val) > 255) ? 2 : 1) * (RGB ? 3 : 1);

        image.data.resize(image.dimensions.x() * image.dimensions.y() * bytes_per_pixel, 0);

        // Skip data in file until a whitespace character is found.
        while (ifs && !std::isspace(ifs.peek())) {
            ifs.ignore();
        }

        // Skip all whitespace until we find non-whtespace.
        while (ifs && std::isspace(ifs.peek())) {
            ifs.ignore();
        }

        ifs.read(reinterpret_cast<char*>(image.data.data()), image.data.size());

        ifs.close();
    }

    const auto getSubImage(uint x, uint y, uint width, uint height, const std::vector<uint8_t>& data);
    uint8_t conv2d(const Eigen::Matrix<uint8_t, 5, 5>& patch,
                   const Eigen::Matrix<int8_t, 5, 5>& kernel,
                   uint8_t normalisation = BAYER_SCALE);
    Pixel getBayerPixel(const Eigen::Matrix<uint8_t, 5, 5>& patch, const BayerPixelType& type);
    Pixel getGrey8Pixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getGRBGPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getRGGBPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getGBRGPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getBGGRPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getGrey16Pixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getRGB3Pixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getYUV24Pixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getYUYVPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getUYVYPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getYUV12Pixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data);
    Pixel getPixel(uint x, uint y, uint width, uint height, const std::vector<uint8_t>& data, const FOURCC& fourcc);
    constexpr FOURCC fourcc(const char (&code)[5]);
    FOURCC getFourCCFromDescription(const std::string& code);
    uint32_t getAravisPixelFormat(const std::string& code);


}  // namespace vision
}  // namespace utility

#endif  // UTILITY_VISION_VISION_H
