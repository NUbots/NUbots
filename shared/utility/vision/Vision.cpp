/*
 * MIT License
 *
 * Copyright (c) 2016 NUbots
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
#include "Vision.hpp"

#include <fmt/format.h>
#include <fstream>
#include <string>

#include "message/input/Image.hpp"

namespace utility::vision {

    void saveImage(const std::string& file, const message::input::Image& image) {
        std::ofstream ofs(file, std::ios::out | std::ios::binary);
        ofs << fmt::format("P6\n{} {}\n255\n", image.dimensions[0], image.dimensions[1]);

        for (size_t row = 0; row < image.dimensions[1]; row++) {
            for (size_t col = 0; col < image.dimensions[0]; col++) {
                Pixel p =
                    getPixel(col, row, image.dimensions[0], image.dimensions[1], image.data, FOURCC(image.format));
                ofs.write(reinterpret_cast<char*>(&p.components.r), sizeof(p.components.r));
                ofs.write(reinterpret_cast<char*>(&p.components.g), sizeof(p.components.g));
                ofs.write(reinterpret_cast<char*>(&p.components.b), sizeof(p.components.b));
            }
        }

        ofs.close();
    }

    auto getSubImage(uint x, uint y, uint width, uint height, const std::vector<uint8_t>& data) {
        // Extract the 5x5 matrix centered at (x, y).
        // Clamped to borders.
        x = x < 2 ? 2 : x > (width - 3) ? width - 3 : x;
        y = y < 2 ? 2 : y > (height - 3) ? height - 3 : y;

        return Eigen::Map<const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(data.data(),
                                                                                                         height,
                                                                                                         width)
            .block<5, 5>(y - 2, x - 2);
    }

    uint8_t conv2d(const Eigen::Matrix<uint8_t, 5, 5>& patch,
                   const Eigen::Matrix<int8_t, 5, 5>& kernel,
                   uint8_t normalisation) {
        int16_t value = patch.cast<int16_t>().cwiseProduct(kernel.cast<int16_t>()).sum();

        if (normalisation == 0) {
            normalisation = 1;
        }

        return static_cast<uint8_t>(std::min(255, std::max(0, value >> normalisation)));
    }

    Pixel getBayerPixel(const Eigen::Matrix<uint8_t, 5, 5>& patch, const BayerPixelType& type) {
        Pixel p;

        switch (type) {
            // Centered on a red pixel
            case BayerPixelType::R: {
                p.components.r = patch(2, 2);
                p.components.g = conv2d(patch, GREEN_AXIAL);
                p.components.b = conv2d(patch, RED_AT_BLUE);
                return p;
            }
            // Centered on a green pixel in a red row
            case BayerPixelType::GR: {
                p.components.r = conv2d(patch, RED_AT_GREEN.transpose());
                p.components.g = patch(2, 2);
                p.components.b = conv2d(patch, RED_AT_GREEN);
                return p;
            }
            // Centered on a green pixel in a blue row
            case BayerPixelType::GB: {
                p.components.r = conv2d(patch, RED_AT_GREEN);
                p.components.g = patch(2, 2);
                p.components.b = conv2d(patch, RED_AT_GREEN.transpose());
                return p;
            }
            // Centered on a blue pixel
            case BayerPixelType::B: {
                p.components.r = conv2d(patch, RED_AT_BLUE);
                p.components.g = conv2d(patch, GREEN_AXIAL);
                p.components.b = patch(2, 2);
                return p;
            }
            default: return p;
        }
    }

    Pixel getGrey8Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // R0 G0 B0 R1 GR B1 R2 GB B2 ...
        const int origin = (y * width + x);

        return {0, 0, data[origin]};
    }

    Pixel getGRBGPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // Col    0 1 2 3 4 5
        // Row 0: G R G R G R ....
        // Row 1: B G B G B G ....
        // Red   pixels are in even rows, but odd  columns
        // Green pixels are in every row, but in the even columns on even rows and the odd columns on odd rows.
        // Blue  pixels are in odd rows,  but even columns

        const Eigen::Matrix<uint8_t, 5, 5> patch = getSubImage(x, y, width, height, data);

        // Work out what pixel type we are
        const int row       = y % 2;
        const int col       = x % 2;
        BayerPixelType type = row != 0   ? col != 0 ? BayerPixelType::GB : BayerPixelType::B
                              : col != 0 ? BayerPixelType::R
                                         : BayerPixelType::GR;

        return getBayerPixel(patch, type);
    }

    Pixel getRGGBPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // Col    0 1 2 3 4 5
        // Row 0: R G R G R G ....
        // Row 1: G B G B G B ....
        // Red   pixels are in even rows, but even columns
        // Green pixels are in every row, but in the odd columns on even rows and the even columns on odd rows.
        // Blue  pixels are in odd rows,  but odd  columns

        Eigen::Matrix<uint8_t, 5, 5> patch = getSubImage(x, y, width, height, data);

        // Work out what pixel type we are
        const int row       = y % 2;
        const int col       = x % 2;
        BayerPixelType type = row != 0   ? col != 0 ? BayerPixelType::B : BayerPixelType::GB
                              : col != 0 ? BayerPixelType::GR
                                         : BayerPixelType::R;

        return getBayerPixel(patch, type);
    }

    Pixel getGBRGPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // Col    0 1 2 3 4 5
        // Row 0: G B G B G B ....
        // Row 1: R G R G R G ....
        // Red   pixels are in odd rows,  but even columns
        // Green pixels are in every row, but in the even columns on even rows and the odd columns on odd rows.
        // Blue  pixels are in even rows, but odd columns

        Eigen::Matrix<uint8_t, 5, 5> patch = getSubImage(x, y, width, height, data);

        // Work out what pixel type we are
        const int row       = y % 2;
        const int col       = x % 2;
        BayerPixelType type = row != 0   ? col != 0 ? BayerPixelType::GR : BayerPixelType::R
                              : col != 0 ? BayerPixelType::B
                                         : BayerPixelType::GB;

        return getBayerPixel(patch, type);
    }

    Pixel getBGGRPixel(uint x, uint y, int width, int height, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // Col    0 1 2 3 4 5
        // Row 0: B G B G B G ....
        // Row 1: G R G R G R ....
        // Red   pixels are in odd rows,  but odd columns
        // Green pixels are in every row, but in the even columns on odd rows and the odd columns on even rows.
        // Blue  pixels are in even rows, but even columns

        Eigen::Matrix<uint8_t, 5, 5> patch = getSubImage(x, y, width, height, data);

        // Work out what pixel type we are
        const int row       = y % 2;
        const int col       = x % 2;
        BayerPixelType type = row != 0   ? col != 0 ? BayerPixelType::R : BayerPixelType::GR
                              : col != 0 ? BayerPixelType::GB
                                         : BayerPixelType::B;

        return getBayerPixel(patch, type);
    }

    Pixel getGrey16Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        const int origin = (y * width + x) * 2;

        return {0, data[origin + 1], data[origin]};
    }

    Pixel getRGB3Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // R0 G0 B0 R1 GR B1 R2 GB B2 ...
        const int origin = (y * width + x) * 3;

        return {data[origin + 0], data[origin + 1], data[origin + 2]};
    }

    Pixel getYUV24Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // U0 Y0 V0 U1 Y1 V1 U2 Y2 V2
        const int origin = (y * width + x) * 3;

        return {data[origin + 1], data[origin + 0], data[origin + 2]};
    }

    Pixel getYUYVPixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // Y U Y V Y U Y V Y U Y V
        const int origin = (y * width + x) * 2;
        const int shift  = (x % 2) * 2;

        // origin = Y, always.
        return {data[origin + 0], data[origin + 1 - shift], data[origin + 3 - shift]};
    }

    Pixel getUYVYPixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // U Y V Y U Y V Y U Y V Y
        const int origin = (y * width + x) * 2;
        const int shift  = (x % 2) * 2;

        // Either    shift = 0 and origin = U
        // Or        shift = 2 and origin = V.
        return {data[origin + 1], data[origin + 0 - shift], data[origin + 2 - shift]};
    }

    Pixel getYUV12Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // U0 Y0 Y1 V0 Y2 Y3 U1 Y4 Y5 V1 Y6 Y7 U2 Y8 Y9 V1 Y10 Y11
        // U0Y0V0 U0Y1V0 U0Y2V0 U0Y3V0 U1Y4V1

        // 4 pixels every 6 bytes.
        // Increment origin in steps of 6.
        // origin will always land on U for the current block of 4 pixels
        // V for the current block of 4 pixels will always be origin + 3
        // Either
        // Components           Data Components   (x, y)
        // U0Y0V0     {data[0], data[1], data[3]} (0, 0)
        // U0Y1V0     {data[0], data[2], data[3]} (1, 0)
        // U0Y2V0     {data[0], data[4], data[3]} (2, 0)
        // U0Y3V0     {data[0], data[5], data[3]} (3, 0)
        // U1Y4V1     {data[6], data[7], data[9]} (4, 0)
        // U1Y5V1     {data[6], data[8], data[9]} (5, 0)
        const uint Y_OFFSET[6] = {1, 2, 4, 5, 7, 8};

        const int origin  = (y * width + (x - (x % 6))) * 6;
        const int shift   = ((x % 6) > 3) ? 6 : 0;
        const int y_shift = Y_OFFSET[x % 4];

        return {data[origin + 0 + shift], data[origin + 0 + y_shift], data[origin + 3 + shift]};
    }

    Pixel getPixel(uint x, uint y, uint width, uint height, const std::vector<uint8_t>& data, const FOURCC& fourcc) {
        switch (fourcc) {
            case GREY: {
                return (getGrey8Pixel(x, y, width, height, data));
            }

            case Y12:
            case Y16: {
                return (getGrey16Pixel(x, y, width, height, data));
            }

            case Y411: {
                return (getYUV12Pixel(x, y, width, height, data));
            }

            case UYVY: {
                return (getUYVYPixel(x, y, width, height, data));
            }

            case YM24: {
                return (getYUV24Pixel(x, y, width, height, data));
            }

            case YUYV: {
                return (getYUYVPixel(x, y, width, height, data));
            }

            case RGB3: {
                return (getRGB3Pixel(x, y, width, height, data));
            }

            case GRBG: {
                return (getGRBGPixel(x, y, width, height, data));
            }

            case RGGB: {
                return (getRGGBPixel(x, y, width, height, data));
            }

            case GBRG: {
                return (getGBRGPixel(x, y, width, height, data));
            }

            case BGGR: {
                return (getBGGRPixel(x, y, width, height, data));
            }


            case GR12:
            case RG12:
            case GB12:
            case BG12:
            case GR16:
            case RG16:
            case GB16:
            case BG16:
            case UNKNOWN:
            default: {
                return {0, 0, 0};
            }
        }
    }

}  // namespace utility::vision
