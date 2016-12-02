#ifndef UTILITY_VISION_FOURCC_H
#define UTILITY_VISION_FOURCC_H

#include <vector>
#include <string>

namespace utility {
    namespace vision {

        enum FOURCC : uint32_t {
            GREY = 0x59455247,
            Y12  = 0x20323159,
            Y16  = 0x20363159,
            GRBG = 0x47425247,
            RGGB = 0x42474752,
            GBRG = 0x47524247,
            BGGR = 0x52474742,
            GR12 = 0x32315247,
            RG12 = 0x32314752,
            GB12 = 0x32314247,
            BG12 = 0x32314742,
            GR16 = 0x36315247,
            RG16 = 0x36314752,
            GB16 = 0x36314247,
            BG16 = 0x36314742,
            Y411 = 0x31313459,
            UYVY = 0x59565955,
            YUYV = 0x56595559,
            YM24 = 0x34324d59,
            RGB3 = 0x33424752,
            JPEG = 0x4745504a,
            UNKNOWN = 0
        };

        struct Pixel {
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
        };

        inline Pixel getGrey8Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data)
        {
            // Asumming pixels are stored as
            // R0 G0 B0 R1 G1 B1 R2 G2 B2 ...
            int origin = (y * width + x);

            return {
                0, 0, 
                data[origin]
            };
        }

        inline Pixel getGrey16Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data)
        {
            int origin = (y * width + x) * 2;

            return {
                0,
                data[origin + 1],
                data[origin]
            };
        }

        inline Pixel getRGB3Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data)
        {
            // Asumming pixels are stored as
            // R0 G0 B0 R1 G1 B1 R2 G2 B2 ...
            int origin = (y * width + x) * 3;

            return {
                data[origin + 0],
                data[origin + 1],
                data[origin + 2]
            };
        }

        inline Pixel getYUV24Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data)
        {
            // Asumming pixels are stored as
            // U0 Y0 V0 U1 Y1 V1 U2 Y2 V2
            int origin = (y * width + x) * 3;

            return {
                data[origin + 1],
                data[origin + 0],
                data[origin + 2]
            };
        }

        inline Pixel getYUYVPixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data)
        {
            // Asumming pixels are stored as
            // Y U Y V Y U Y V Y U Y V
            int origin = (y * width + x) * 2;
            int shift = (x % 2) * 2;

            // origin = Y, always.
            return {
                data[origin + 0],
                data[origin + 1 - shift],
                data[origin + 3 - shift]
            };
        }

        inline Pixel getUYVYPixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data)
        {
            // Asumming pixels are stored as
            // U Y V Y U Y V Y U Y V Y
            int origin = (y * width + x) * 2;
            int shift = (x % 2) * 2;

            // Either    shift = 0 and origin = U
            // Or        shift = 2 and origin = V.
            return {
                data[origin + 1],
                data[origin + 0 - shift],
                data[origin + 2 - shift]
            };
        }

        inline Pixel getYUV12Pixel(uint x, uint y, int width, int /*height*/, const std::vector<uint8_t>& data)
        {
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

            return {
                data[origin + 0 +   shift],
                data[origin + 0 + y_shift],
                data[origin + 3 +   shift]
            };
        }

        inline Pixel getPixel(uint x, uint y, uint width, uint height, const std::vector<uint8_t>& data, const FOURCC& fourcc)
        {
            switch (fourcc)
            {
                case GREY:
                {
                    return(getGrey8Pixel(x, y, width, height, data));
                }

                case Y12:
                case Y16:
                {
                    return(getGrey16Pixel(x, y, width, height, data));
                }

                case Y411:
                {
                    return(getYUV12Pixel(x, y, width, height, data));
                }

                case UYVY:
                {
                    return(getUYVYPixel(x, y, width, height, data));
                }

                case YM24:
                {
                    return(getYUV24Pixel(x, y, width, height, data));
                }

                case YUYV:
                {
                    return(getYUYVPixel(x, y, width, height, data));
                }

                case RGB3:
                {
                    return(getRGB3Pixel(x, y, width, height, data));
                }

                // Bayer pixels will require demosaicing.
                // This is a "future me" problem.
                case GRBG:
                case RGGB:
                case GBRG:
                case BGGR:
                case GR12:
                case RG12:
                case GB12:
                case BG12:
                case GR16:
                case RG16:
                case GB16:
                case BG16:
                case UNKNOWN:
                default:
                {
                    return {0, 0, 0};
                }
            }
        }

        inline constexpr FOURCC fourcc(const char (&code)[5]) {
            uint32_t cc = (((code[0]) & 255) | (((code[1]) & 255) << 8) | (((code[2]) & 255) << 16) | (((code[3]) & 255) << 24));
            return((FOURCC)cc);
        }

        inline FOURCC getFourCCFromDescription(const std::string& code) {
            if (code.compare("Mono8") == 0)
            {
                return(fourcc("GREY"));
            }

            else if (code.compare("Mono12Packed") == 0)
            {
                return(fourcc("Y12 "));
            }

            else if (code.compare("Mono12p") == 0)
            {
                return(fourcc("Y12 "));
            }

            else if (code.compare("Mono16") == 0)
            {
                return(fourcc("Y16 "));
            }

            else if (code.compare("BayerGR8") == 0)
            {
                return(fourcc("GRBG"));
            }

            else if (code.compare("BayerRG8") == 0)
            {
                return(fourcc("RGGB"));
            }

            else if (code.compare("BayerGB8") == 0)
            {
                return(fourcc("GBRG"));
            }

            else if (code.compare("BayerBG8") == 0)
            {
                return(fourcc("BGGR"));
            }

            else if (code.compare("BayerGR12p") == 0)
            {
                return(fourcc("GR12"));
            }

            else if (code.compare("BayerRG12p") == 0)
            {
                return(fourcc("RG12"));
            }

            else if (code.compare("BayerGB12p") == 0)
            {
                return(fourcc("GB12"));
            }

            else if (code.compare("BayerBG12p") == 0)
            {
                return(fourcc("BG12"));
            }

            else if (code.compare("BayerGR12Packed") == 0)
            {
                return(fourcc("GR12"));
            }

            else if (code.compare("BayerRG12Packed") == 0)
            {
                return(fourcc("RG12"));
            }

            else if (code.compare("BayerGB12Packed") == 0)
            {
                return(fourcc("GB12"));
            }

            else if (code.compare("BayerBG12Packed") == 0)
            {
                return(fourcc("BG12"));
            }

            else if (code.compare("BayerGR16") == 0)
            {
                return(fourcc("GR16"));
            }

            else if (code.compare("BayerRG16") == 0)
            {
                return(fourcc("RG16"));
            }

            else if (code.compare("BayerGB16") == 0)
            {
                return(fourcc("GB16"));
            }

            else if (code.compare("BayerBG16") == 0)
            {
                return(fourcc("BG16"));
            }

            else if (code.compare("YCbCr411_8_CbYYCrYY") == 0)
            {
                return(fourcc("Y411"));
            }

            else if (code.compare("YCbCr422_8_CbYCrY") == 0)
            {
                return(fourcc("UYVY"));
            }

            else if (code.compare("YCbCr8_CbYCr") == 0)
            {
                return(fourcc("YM24"));
            }

            else if (code.compare("YUYV") == 0)
            {
                return(fourcc("YUYV"));
            }

            else if (code.compare("RGB8") == 0)
            {
                return(fourcc("RGB3"));
            }

            else
            {
                return(FOURCC::UNKNOWN);
            }
        }
    }
}

#endif //COMPANION_FOURCC_H
