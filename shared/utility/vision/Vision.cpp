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
#include "Vision.h"

#include <fmt/format.h>
#include <fstream>
#include <string>

#include "message/input/Image.h"

namespace utility {
namespace vision {

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

    const auto getSubImage(uint x, uint y, uint width, uint height, const std::vector<uint8_t>& data) {
        // Extract the 5x5 matrix centered at (x, y).
        // Clamped to borders.
        x = x < 2 ? 2 : x > (width - 3) ? width - 3 : x;
        y = y < 2 ? 2 : y > (height - 3) ? height - 3 : y;

        return Eigen::Map<const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                   data.data(), height, width)
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
        int origin = (y * width + x);

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

        Eigen::Matrix<uint8_t, 5, 5> patch = getSubImage(x, y, width, height, data);

        // Work out what pixel type we are
        const int row = y % 2;
        const int col = x % 2;
        BayerPixelType type =
            row ? col ? BayerPixelType::GB : BayerPixelType::B : col ? BayerPixelType::R : BayerPixelType::GR;

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
        const int row = y % 2;
        const int col = x % 2;
        BayerPixelType type =
            row ? col ? BayerPixelType::B : BayerPixelType::GB : col ? BayerPixelType::GR : BayerPixelType::R;

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
        const int row = y % 2;
        const int col = x % 2;
        BayerPixelType type =
            row ? col ? BayerPixelType::GR : BayerPixelType::R : col ? BayerPixelType::B : BayerPixelType::GB;

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
        const int row = y % 2;
        const int col = x % 2;
        BayerPixelType type =
            row ? col ? BayerPixelType::R : BayerPixelType::GR : col ? BayerPixelType::GB : BayerPixelType::B;

        return getBayerPixel(patch, type);
    }

    Pixel getGrey16Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        int origin = (y * width + x) * 2;

        return {0, data[origin + 1], data[origin]};
    }

    Pixel getRGB3Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // R0 G0 B0 R1 GR B1 R2 GB B2 ...
        int origin = (y * width + x) * 3;

        return {data[origin + 0], data[origin + 1], data[origin + 2]};
    }

    Pixel getYUV24Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // U0 Y0 V0 U1 Y1 V1 U2 Y2 V2
        int origin = (y * width + x) * 3;

        return {data[origin + 1], data[origin + 0], data[origin + 2]};
    }

    Pixel getYUYVPixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // Y U Y V Y U Y V Y U Y V
        int origin = (y * width + x) * 2;
        int shift  = (x % 2) * 2;

        // origin = Y, always.
        return {data[origin + 0], data[origin + 1 - shift], data[origin + 3 - shift]};
    }

    Pixel getUYVYPixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data) {
        // Asumming pixels are stored as
        // U Y V Y U Y V Y U Y V Y
        int origin = (y * width + x) * 2;
        int shift  = (x % 2) * 2;

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

        int origin  = (y * width + (x - (x % 6))) * 6;
        int shift   = ((x % 6) > 3) ? 6 : 0;
        int y_shift = Y_OFFSET[x % 4];

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
            default: { return {0, 0, 0}; }
        }
    }

    constexpr FOURCC fourcc(const char (&code)[5]) {
        uint32_t cc =
            (((code[0]) & 255) | (((code[1]) & 255) << 8) | (((code[2]) & 255) << 16) | (((code[3]) & 255) << 24));
        return ((FOURCC) cc);
    }

    FOURCC getFourCCFromDescription(const std::string& code) {
        if (code.compare("Mono8") == 0) {
            return (fourcc("GREY"));
        }

        else if (code.compare("Mono12Packed") == 0) {
            return (fourcc("Y12 "));
        }

        else if (code.compare("Mono12p") == 0) {
            return (fourcc("Y12 "));
        }

        else if (code.compare("Mono16") == 0) {
            return (fourcc("Y16 "));
        }

        else if (code.compare("BayerGR8") == 0) {
            return (fourcc("GRBG"));
        }

        else if (code.compare("BayerRG8") == 0) {
            return (fourcc("RGGB"));
        }

        else if (code.compare("BayerGB8") == 0) {
            return (fourcc("GBRG"));
        }

        else if (code.compare("BayerBG8") == 0) {
            return (fourcc("BGGR"));
        }

        else if (code.compare("BayerGR12p") == 0) {
            return (fourcc("GR12"));
        }

        else if (code.compare("BayerRG12p") == 0) {
            return (fourcc("RG12"));
        }

        else if (code.compare("BayerGB12p") == 0) {
            return (fourcc("GB12"));
        }

        else if (code.compare("BayerBG12p") == 0) {
            return (fourcc("BG12"));
        }

        else if (code.compare("BayerGR12Packed") == 0) {
            return (fourcc("GR12"));
        }

        else if (code.compare("BayerRG12Packed") == 0) {
            return (fourcc("RG12"));
        }

        else if (code.compare("BayerGB12Packed") == 0) {
            return (fourcc("GB12"));
        }

        else if (code.compare("BayerBG12Packed") == 0) {
            return (fourcc("BG12"));
        }

        else if (code.compare("BayerGR16") == 0) {
            return (fourcc("GR16"));
        }

        else if (code.compare("BayerRG16") == 0) {
            return (fourcc("RG16"));
        }

        else if (code.compare("BayerGB16") == 0) {
            return (fourcc("GB16"));
        }

        else if (code.compare("BayerBG16") == 0) {
            return (fourcc("BG16"));
        }

        else if (code.compare("YCbCr411_8_CbYYCrYY") == 0) {
            return (fourcc("Y411"));
        }

        else if (code.compare("YCbCr422_8_CbYCrY") == 0) {
            return (fourcc("UYVY"));
        }

        else if (code.compare("YCbCr8_CbYCr") == 0) {
            return (fourcc("YM24"));
        }

        else if (code.compare("YUYV") == 0) {
            return (fourcc("YUYV"));
        }

        else if (code.compare("RGB8") == 0) {
            return (fourcc("RGB3"));
        }

        else {
            return (FOURCC::UNKNOWN);
        }
    }

    uint32_t getAravisPixelFormat(const std::string& code) {
        if (code.compare("Mono8") == 0) {
            return ARV_PIXEL_FORMAT_MONO_8;
        }

        else if (code.compare("Mono12Packed") == 0) {
            return ARV_PIXEL_FORMAT_MONO_12_PACKED;
        }

        else if (code.compare("Mono12p") == 0) {
            return ARV_PIXEL_FORMAT_MONO_12;
        }

        else if (code.compare("Mono16") == 0) {
            return ARV_PIXEL_FORMAT_MONO_16;
        }

        else if (code.compare("BayerGR8") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_GR_8;
        }

        else if (code.compare("BayerRG8") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_RG_8;
        }

        else if (code.compare("BayerGB8") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_GB_8;
        }

        else if (code.compare("BayerBG8") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_BG_8;
        }

        else if (code.compare("BayerGR12p") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_GR_12;
        }

        else if (code.compare("BayerRG12p") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_RG_12;
        }

        else if (code.compare("BayerGB12p") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_GB_12;
        }

        else if (code.compare("BayerBG12p") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_BG_12;
        }

        else if (code.compare("BayerGR12Packed") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_GR_12_PACKED;
        }

        else if (code.compare("BayerRG12Packed") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_RG_12_PACKED;
        }

        else if (code.compare("BayerGB12Packed") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_GB_12_PACKED;
        }

        else if (code.compare("BayerBG12Packed") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_BG_12_PACKED;
        }

        else if (code.compare("BayerGR16") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_GR_16;
        }

        else if (code.compare("BayerRG16") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_RG_16;
        }

        else if (code.compare("BayerGB16") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_GB_16;
        }

        else if (code.compare("BayerBG16") == 0) {
            return ARV_PIXEL_FORMAT_BAYER_BG_16;
        }

        else if (code.compare("YCbCr411_8_CbYYCrYY") == 0) {
            return ARV_PIXEL_FORMAT_YUV_411_PACKED;
        }

        else if (code.compare("YCbCr422_8_CbYCrY") == 0) {
            return ARV_PIXEL_FORMAT_YUV_422_PACKED;
        }

        else if (code.compare("YCbCr8_CbYCr") == 0) {
            return ARV_PIXEL_FORMAT_YUV_444_PACKED;
        }

        else if (code.compare("YUYV") == 0) {
            return ARV_PIXEL_FORMAT_YUV_422_YUYV_PACKED;
        }

        else if (code.compare("RGB8") == 0) {
            return ARV_PIXEL_FORMAT_RGB_8_PACKED;
        }

        else {
            return 0;
        }
    }

}  // namespace vision
}  // namespace utility
