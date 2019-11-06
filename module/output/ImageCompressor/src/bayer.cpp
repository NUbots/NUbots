/*
 * 1394-Based Digital Camera Control Library
 *
 * Bayer pattern decoding functions
 *
 * Written by Damien Douxchamps and Frederic Devernay
 * The original VNG and AHD Bayer decoding are from Dave Coffin's DCRAW.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "bayer.h"

#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "utility/vision/Vision.h"

namespace module {
namespace output {

    template <typename T>
    constexpr T clip(const T& v) {
        return v < 0 ? 0 : v > 255 ? 255 : v;
    }

    void clear_borders(uint8_t* rgb, int sx, int sy, int w) {
        int i, j;
        // black edges are added with a width w:
        i = 3 * sx * w - 1;
        j = 3 * sx * sy - 1;
        while (i >= 0) {
            rgb[i--] = 0;
            rgb[j--] = 0;
        }

        int low = sx * (w - 1) * 3 - 1 + w * 3;
        i       = low + sx * (sy - w * 2 + 1) * 3;
        while (i > low) {
            j = 6 * w;
            while (j > 0) {
                rgb[i--] = 0;
                j--;
            }
            i -= (sx - 2 * w) * 3;
        }
    }


    /**************************************************************
     *     Color conversion functions for cameras that can        *
     * output raw-Bayer pattern images, such as some Basler and   *
     * Point Grey camera. Most of the algos presented here come   *
     * from http://www-ise.stanford.edu/~tingchen/ and have been  *
     * converted from Matlab to C and extended to all elementary  *
     * patterns.                                                  *
     **************************************************************/

    /* OpenCV's Bayer decoding */
    std::vector<uint8_t> debayer_bilinear(const uint8_t* bayer, int sx, int sy, uint32_t tile) {
        std::vector<uint8_t> output(sx * sy * 3);
        auto rgb = output.data();

        const int bayerStep = sx;
        const int rgbStep   = 3 * sx;
        int width           = sx;
        int height          = sy;
        /*
           the two letters  of the OpenCV name are respectively
           the 4th and 3rd letters from the blinky name,
           and we also have to switch R and B (OpenCV is BGR)

           CV_BayerBG2BGR <-> utility::vision::fourcc("BGGR")
           CV_BayerGB2BGR <-> utility::vision::fourcc("GBRG")
           CV_BayerGR2BGR <-> utility::vision::fourcc("GRBG")

           int blue = tile == CV_BayerBG2BGR || tile == CV_BayerGB2BGR ? -1 : 1;
           int start_with_green = tile == CV_BayerGB2BGR || tile == CV_BayerGR2BGR;
         */
        int blue = tile == utility::vision::fourcc("BGGR") || tile == utility::vision::fourcc("GBRG") ? -1 : 1;
        int start_with_green = tile == utility::vision::fourcc("GBRG") || tile == utility::vision::fourcc("GRBG");

        clear_borders(rgb, sx, sy, 1);
        rgb += rgbStep + 3 + 1;
        height -= 2;
        width -= 2;

        for (; height--; bayer += bayerStep, rgb += rgbStep) {
            int t0, t1;
            const uint8_t* bayerEnd = bayer + width;

            if (start_with_green) {
                /* OpenCV has a bug in the next line, which was
                   t0 = (bayer[0] + bayer[bayerStep * 2] + 1) >> 1; */
                t0         = (bayer[1] + bayer[bayerStep * 2 + 1] + 1) >> 1;
                t1         = (bayer[bayerStep] + bayer[bayerStep + 2] + 1) >> 1;
                rgb[-blue] = (uint8_t) t0;
                rgb[0]     = bayer[bayerStep + 1];
                rgb[blue]  = (uint8_t) t1;
                bayer++;
                rgb += 3;
            }

            if (blue > 0) {
                for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                    t0      = (bayer[0] + bayer[2] + bayer[bayerStep * 2] + bayer[bayerStep * 2 + 2] + 2) >> 2;
                    t1      = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] + 2) >> 2;
                    rgb[-1] = (uint8_t) t0;
                    rgb[0]  = (uint8_t) t1;
                    rgb[1]  = bayer[bayerStep + 1];

                    t0     = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
                    t1     = (bayer[bayerStep + 1] + bayer[bayerStep + 3] + 1) >> 1;
                    rgb[2] = (uint8_t) t0;
                    rgb[3] = bayer[bayerStep + 2];
                    rgb[4] = (uint8_t) t1;
                }
            }
            else {
                for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                    t0      = (bayer[0] + bayer[2] + bayer[bayerStep * 2] + bayer[bayerStep * 2 + 2] + 2) >> 2;
                    t1      = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] + 2) >> 2;
                    rgb[1]  = (uint8_t) t0;
                    rgb[0]  = (uint8_t) t1;
                    rgb[-1] = bayer[bayerStep + 1];

                    t0     = (bayer[2] + bayer[bayerStep * 2 + 2] + 1) >> 1;
                    t1     = (bayer[bayerStep + 1] + bayer[bayerStep + 3] + 1) >> 1;
                    rgb[4] = (uint8_t) t0;
                    rgb[3] = bayer[bayerStep + 2];
                    rgb[2] = (uint8_t) t1;
                }
            }

            if (bayer < bayerEnd) {
                t0         = (bayer[0] + bayer[2] + bayer[bayerStep * 2] + bayer[bayerStep * 2 + 2] + 2) >> 2;
                t1         = (bayer[1] + bayer[bayerStep] + bayer[bayerStep + 2] + bayer[bayerStep * 2 + 1] + 2) >> 2;
                rgb[-blue] = (uint8_t) t0;
                rgb[0]     = (uint8_t) t1;
                rgb[blue]  = bayer[bayerStep + 1];
                bayer++;
                rgb += 3;
            }

            bayer -= width;
            rgb -= width * 3;

            blue             = -blue;
            start_with_green = !start_with_green;
        }
        return output;
    }

    /* High-Quality Linear Interpolation For Demosaicing Of
       Bayer-Patterned Color Images, by Henrique S. Malvar, Li-wei He, and
       Ross Cutler, in ICASSP'04 */
    std::vector<uint8_t> debayer_hqlinear(const uint8_t* bayer, int sx, int sy, int tile) {
        std::vector<uint8_t> output(sx * sy * 3);
        auto rgb = output.data();

        const int bayerStep = sx;
        const int rgbStep   = 3 * sx;
        int width           = sx;
        int height          = sy;
        int blue = tile == utility::vision::fourcc("BGGR") || tile == utility::vision::fourcc("GBRG") ? -1 : 1;
        int start_with_green = tile == utility::vision::fourcc("GBRG") || tile == utility::vision::fourcc("GRBG");

        clear_borders(rgb, sx, sy, 2);
        rgb += 2 * rgbStep + 6 + 1;
        height -= 4;
        width -= 4;

        /* We begin with a (+1 line,+1 column) offset with respect to bilinear decoding, so start_with_green is the
         * same, but blue is opposite */
        blue = -blue;

        for (; height--; bayer += bayerStep, rgb += rgbStep) {
            int t0, t1;
            const uint8_t* bayerEnd = bayer + width;
            const int bayerStep2    = bayerStep * 2;
            const int bayerStep3    = bayerStep * 3;
            const int bayerStep4    = bayerStep * 4;

            if (start_with_green) {
                /* at green pixel */
                rgb[0] = bayer[bayerStep2 + 2];
                t0     = rgb[0] * 5 + ((bayer[bayerStep + 2] + bayer[bayerStep3 + 2]) << 2) - bayer[2]
                     - bayer[bayerStep + 1] - bayer[bayerStep + 3] - bayer[bayerStep3 + 1] - bayer[bayerStep3 + 3]
                     - bayer[bayerStep4 + 2] + ((bayer[bayerStep2] + bayer[bayerStep2 + 4] + 1) >> 1);
                t1 = rgb[0] * 5 + ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]) << 2) - bayer[bayerStep2]
                     - bayer[bayerStep + 1] - bayer[bayerStep + 3] - bayer[bayerStep3 + 1] - bayer[bayerStep3 + 3]
                     - bayer[bayerStep2 + 4] + ((bayer[2] + bayer[bayerStep4 + 2] + 1) >> 1);
                t0         = (t0 + 4) >> 3;
                rgb[-blue] = clip(t0);
                t1         = (t1 + 4) >> 3;
                rgb[blue]  = clip(t1);
                bayer++;
                rgb += 3;
            }

            if (blue > 0) {
                for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                    /* B at B */
                    rgb[1] = bayer[bayerStep2 + 2];
                    /* R at B */
                    t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] + bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3])
                          << 1)
                         - (((bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2]) * 3 + 1)
                            >> 1)
                         + rgb[1] * 6;
                    /* G at B */
                    t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2])
                          << 1)
                         - (bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
                         + (rgb[1] << 2);
                    t0      = (t0 + 4) >> 3;
                    rgb[-1] = clip(t0);
                    t1      = (t1 + 4) >> 3;
                    rgb[0]  = clip(t1);
                    /* at green pixel */
                    rgb[3] = bayer[bayerStep2 + 3];
                    t0     = rgb[3] * 5 + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2) - bayer[3]
                         - bayer[bayerStep + 2] - bayer[bayerStep + 4] - bayer[bayerStep3 + 2] - bayer[bayerStep3 + 4]
                         - bayer[bayerStep4 + 3] + ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] + 1) >> 1);
                    t1 = rgb[3] * 5 + ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2) - bayer[bayerStep2 + 1]
                         - bayer[bayerStep + 2] - bayer[bayerStep + 4] - bayer[bayerStep3 + 2] - bayer[bayerStep3 + 4]
                         - bayer[bayerStep2 + 5] + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
                    t0     = (t0 + 4) >> 3;
                    rgb[2] = clip(t0);
                    t1     = (t1 + 4) >> 3;
                    rgb[4] = clip(t1);
                }
            }
            else {
                for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                    /* R at R */
                    rgb[-1] = bayer[bayerStep2 + 2];
                    /* B at R */
                    t0 = ((bayer[bayerStep + 1] + bayer[bayerStep + 3] + bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3])
                          << 1)
                         - (((bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2]) * 3 + 1)
                            >> 1)
                         + rgb[-1] * 6;
                    /* G at R */
                    t1 = ((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3]
                           + bayer[bayerStep * 3 + 2])
                          << 1)
                         - (bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
                         + (rgb[-1] << 2);
                    t0     = (t0 + 4) >> 3;
                    rgb[1] = clip(t0);
                    t1     = (t1 + 4) >> 3;
                    rgb[0] = clip(t1);

                    /* at green pixel */
                    rgb[3] = bayer[bayerStep2 + 3];
                    t0     = rgb[3] * 5 + ((bayer[bayerStep + 3] + bayer[bayerStep3 + 3]) << 2) - bayer[3]
                         - bayer[bayerStep + 2] - bayer[bayerStep + 4] - bayer[bayerStep3 + 2] - bayer[bayerStep3 + 4]
                         - bayer[bayerStep4 + 3] + ((bayer[bayerStep2 + 1] + bayer[bayerStep2 + 5] + 1) >> 1);
                    t1 = rgb[3] * 5 + ((bayer[bayerStep2 + 2] + bayer[bayerStep2 + 4]) << 2) - bayer[bayerStep2 + 1]
                         - bayer[bayerStep + 2] - bayer[bayerStep + 4] - bayer[bayerStep3 + 2] - bayer[bayerStep3 + 4]
                         - bayer[bayerStep2 + 5] + ((bayer[3] + bayer[bayerStep4 + 3] + 1) >> 1);
                    t0     = (t0 + 4) >> 3;
                    rgb[4] = clip(t0);
                    t1     = (t1 + 4) >> 3;
                    rgb[2] = clip(t1);
                }
            }

            if (bayer < bayerEnd) {
                /* B at B */
                rgb[blue] = bayer[bayerStep2 + 2];
                /* R at B */
                t0 =
                    ((bayer[bayerStep + 1] + bayer[bayerStep + 3] + bayer[bayerStep3 + 1] + bayer[bayerStep3 + 3]) << 1)
                    - (((bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2]) * 3 + 1) >> 1)
                    + rgb[blue] * 6;
                /* G at B */
                t1 = (((bayer[bayerStep + 2] + bayer[bayerStep2 + 1] + bayer[bayerStep2 + 3] + bayer[bayerStep3 + 2]))
                      << 1)
                     - (bayer[2] + bayer[bayerStep2] + bayer[bayerStep2 + 4] + bayer[bayerStep4 + 2])
                     + (rgb[blue] << 2);
                t0         = (t0 + 4) >> 3;
                rgb[-blue] = clip(t0);
                t1         = (t1 + 4) >> 3;
                rgb[0]     = clip(t1);
                bayer++;
                rgb += 3;
            }

            bayer -= width;
            rgb -= width * 3;

            blue             = -blue;
            start_with_green = !start_with_green;
        }

        return output;
    }

    /* coriander's Bayer decoding */
    /* Edge Sensing Interpolation II from http://www-ise.stanford.edu/~tingchen/ */
    /*   (Laroche,Claude A.  "Apparatus and method for adaptively
         interpolating a full color image utilizing chrominance gradients"
         U.S. Patent 5,373,322) */
    std::vector<uint8_t> debayer_edgesense(const uint8_t* bayer, int sx, int sy, int tile) {
        std::vector<uint8_t> output(sx * sy * 3);
        auto rgb = output.data();

        uint8_t *outR, *outG, *outB;
        register int i3, j3, base;
        int i, j;
        int dh, dv;
        int tmp;
        int sx3 = sx * 3;

        // sx and sy should be even
        switch (tile) {
            case utility::vision::fourcc("GRBG"):
            case utility::vision::fourcc("BGGR"):
                outR = &rgb[0];
                outG = &rgb[1];
                outB = &rgb[2];
                break;
            case utility::vision::fourcc("GBRG"):
            case utility::vision::fourcc("RGGB"):
                outR = &rgb[2];
                outG = &rgb[1];
                outB = &rgb[0];
                break;
            default: throw new std::runtime_error("Invalid bayer type");
        }

        switch (tile) {
            case utility::vision::fourcc("GRBG"):  //---------------------------------------------------------
            case utility::vision::fourcc("GBRG"):
                // copy original RGB data to output images
                for (i = 0, i3 = 0; i < sy * sx; i += (sx << 1), i3 += (sx3 << 1)) {
                    for (j = 0, j3 = 0; j < sx; j += 2, j3 += 6) {
                        base                 = i3 + j3;
                        outG[base]           = bayer[i + j];
                        outG[base + sx3 + 3] = bayer[i + j + sx + 1];
                        outR[base + 3]       = bayer[i + j + 1];
                        outB[base + sx3]     = bayer[i + j + sx];
                    }
                }
                // process GREEN channel
                for (i3 = 3 * sx3; i3 < (sy - 2) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 6; j3 < sx3 - 9; j3 += 6) {
                        base = i3 + j3;
                        dh   = abs(((outB[base - 6] + outB[base + 6]) >> 1) - outB[base]);
                        dv   = abs(((outB[base - (sx3 << 1)] + outB[base + (sx3 << 1)]) >> 1) - outB[base]);
                        tmp  = (((outG[base - 3] + outG[base + 3]) >> 1) * (dh <= dv)
                               + ((outG[base - sx3] + outG[base + sx3]) >> 1) * (dh > dv));
                        // tmp = (dh==dv) ? tmp>>1 : tmp;
                        outG[base] = clip(tmp);
                    }
                }

                for (i3 = 2 * sx3; i3 < (sy - 3) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 9; j3 < sx3 - 6; j3 += 6) {
                        base = i3 + j3;
                        dh   = abs(((outR[base - 6] + outR[base + 6]) >> 1) - outR[base]);
                        dv   = abs(((outR[base - (sx3 << 1)] + outR[base + (sx3 << 1)]) >> 1) - outR[base]);
                        tmp  = (((outG[base - 3] + outG[base + 3]) >> 1) * (dh <= dv)
                               + ((outG[base - sx3] + outG[base + sx3]) >> 1) * (dh > dv));
                        // tmp = (dh==dv) ? tmp>>1 : tmp;
                        outG[base] = clip(tmp);
                    }
                }
                // process RED channel
                for (i3 = 0; i3 < (sy - 1) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 6; j3 < sx3 - 3; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base] + ((outR[base - 3] - outG[base - 3] + outR[base + 3] - outG[base + 3]) >> 1);
                        outR[base] = clip(tmp);
                    }
                }
                for (i3 = sx3; i3 < (sy - 2) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 3; j3 < sx3; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base]
                              + ((outR[base - sx3] - outG[base - sx3] + outR[base + sx3] - outG[base + sx3]) >> 1);
                        outR[base] = clip(tmp);
                    }
                    for (j3 = 6; j3 < sx3 - 3; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base]
                              + ((outR[base - sx3 - 3] - outG[base - sx3 - 3] + outR[base - sx3 + 3]
                                  - outG[base - sx3 + 3] + outR[base + sx3 - 3] - outG[base + sx3 - 3]
                                  + outR[base + sx3 + 3] - outG[base + sx3 + 3])
                                 >> 2);
                        outR[base] = clip(tmp);
                    }
                }

                // process BLUE channel
                for (i3 = sx3; i3 < sy * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 3; j3 < sx3 - 6; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base] + ((outB[base - 3] - outG[base - 3] + outB[base + 3] - outG[base + 3]) >> 1);
                        outB[base] = clip(tmp);
                    }
                }
                for (i3 = 2 * sx3; i3 < (sy - 1) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 0; j3 < sx3 - 3; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base]
                              + ((outB[base - sx3] - outG[base - sx3] + outB[base + sx3] - outG[base + sx3]) >> 1);
                        outB[base] = clip(tmp);
                    }
                    for (j3 = 3; j3 < sx3 - 6; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base]
                              + ((outB[base - sx3 - 3] - outG[base - sx3 - 3] + outB[base - sx3 + 3]
                                  - outG[base - sx3 + 3] + outB[base + sx3 - 3] - outG[base + sx3 - 3]
                                  + outB[base + sx3 + 3] - outG[base + sx3 + 3])
                                 >> 2);
                        outB[base] = clip(tmp);
                    }
                }
                break;

            case utility::vision::fourcc("BGGR"):  //---------------------------------------------------------
            case utility::vision::fourcc("RGGB"):
                // copy original RGB data to output images
                for (i = 0, i3 = 0; i < sy * sx; i += (sx << 1), i3 += (sx3 << 1)) {
                    for (j = 0, j3 = 0; j < sx; j += 2, j3 += 6) {
                        base                 = i3 + j3;
                        outB[base]           = bayer[i + j];
                        outR[base + sx3 + 3] = bayer[i + sx + (j + 1)];
                        outG[base + 3]       = bayer[i + j + 1];
                        outG[base + sx3]     = bayer[i + sx + j];
                    }
                }
                // process GREEN channel
                for (i3 = 2 * sx3; i3 < (sy - 2) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 6; j3 < sx3 - 9; j3 += 6) {
                        base = i3 + j3;
                        dh   = abs(((outB[base - 6] + outB[base + 6]) >> 1) - outB[base]);
                        dv   = abs(((outB[base - (sx3 << 1)] + outB[base + (sx3 << 1)]) >> 1) - outB[base]);
                        tmp  = (((outG[base - 3] + outG[base + 3]) >> 1) * (dh <= dv)
                               + ((outG[base - sx3] + outG[base + sx3]) >> 1) * (dh > dv));
                        // tmp = (dh==dv) ? tmp>>1 : tmp;
                        outG[base] = clip(tmp);
                    }
                }
                for (i3 = 3 * sx3; i3 < (sy - 3) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 9; j3 < sx3 - 6; j3 += 6) {
                        base = i3 + j3;
                        dh   = abs(((outR[base - 6] + outR[base + 6]) >> 1) - outR[base]);
                        dv   = abs(((outR[base - (sx3 << 1)] + outR[base + (sx3 << 1)]) >> 1) - outR[base]);
                        tmp  = (((outG[base - 3] + outG[base + 3]) >> 1) * (dh <= dv)
                               + ((outG[base - sx3] + outG[base + sx3]) >> 1) * (dh > dv));
                        // tmp = (dh==dv) ? tmp>>1 : tmp;
                        outG[base] = clip(tmp);
                    }
                }
                // process RED channel
                for (i3 = sx3; i3 < (sy - 1) * sx3; i3 += (sx3 << 1)) {  // G-points (1/2)
                    for (j3 = 6; j3 < sx3 - 3; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base] + ((outR[base - 3] - outG[base - 3] + outR[base + 3] - outG[base + 3]) >> 1);
                        outR[base] = clip(tmp);
                    }
                }
                for (i3 = 2 * sx3; i3 < (sy - 2) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 3; j3 < sx3; j3 += 6) {  // G-points (2/2)
                        base = i3 + j3;
                        tmp  = outG[base]
                              + ((outR[base - sx3] - outG[base - sx3] + outR[base + sx3] - outG[base + sx3]) >> 1);
                        outR[base] = clip(tmp);
                    }
                    for (j3 = 6; j3 < sx3 - 3; j3 += 6) {  // B-points
                        base = i3 + j3;
                        tmp  = outG[base]
                              + ((outR[base - sx3 - 3] - outG[base - sx3 - 3] + outR[base - sx3 + 3]
                                  - outG[base - sx3 + 3] + outR[base + sx3 - 3] - outG[base + sx3 - 3]
                                  + outR[base + sx3 + 3] - outG[base + sx3 + 3])
                                 >> 2);
                        outR[base] = clip(tmp);
                    }
                }

                // process BLUE channel
                for (i = 0, i3 = 0; i < sy * sx; i += (sx << 1), i3 += (sx3 << 1)) {
                    for (j = 1, j3 = 3; j < sx - 2; j += 2, j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base] + ((outB[base - 3] - outG[base - 3] + outB[base + 3] - outG[base + 3]) >> 1);
                        outB[base] = clip(tmp);
                    }
                }
                for (i3 = sx3; i3 < (sy - 1) * sx3; i3 += (sx3 << 1)) {
                    for (j3 = 0; j3 < sx3 - 3; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base]
                              + ((outB[base - sx3] - outG[base - sx3] + outB[base + sx3] - outG[base + sx3]) >> 1);
                        outB[base] = clip(tmp);
                    }
                    for (j3 = 3; j3 < sx3 - 6; j3 += 6) {
                        base = i3 + j3;
                        tmp  = outG[base]
                              + ((outB[base - sx3 - 3] - outG[base - sx3 - 3] + outB[base - sx3 + 3]
                                  - outG[base - sx3 + 3] + outB[base + sx3 - 3] - outG[base + sx3 - 3]
                                  + outB[base + sx3 + 3] - outG[base + sx3 + 3])
                                 >> 2);
                        outB[base] = clip(tmp);
                    }
                }
                break;
        }

        clear_borders(rgb, sx, sy, 3);

        return output;
    }

    /* this is the method used inside AVT cameras. See AVT docs. */
    std::vector<uint8_t> debayer_simple(const uint8_t* bayer, int sx, int sy, int tile) {
        std::vector<uint8_t> output(sx * sy * 3);
        auto rgb = output.data();

        const int bayerStep = sx;
        const int rgbStep   = 3 * sx;
        int width           = sx;
        int height          = sy;
        int blue = tile == utility::vision::fourcc("BGGR") || tile == utility::vision::fourcc("GBRG") ? -1 : 1;
        int start_with_green = tile == utility::vision::fourcc("GBRG") || tile == utility::vision::fourcc("GRBG");
        int i, imax, iinc;

        /* add black border */
        imax = sx * sy * 3;
        for (i = sx * (sy - 1) * 3; i < imax; i++) {
            rgb[i] = 0;
        }
        iinc = (sx - 1) * 3;
        for (i = (sx - 1) * 3; i < imax; i += iinc) {
            rgb[i++] = 0;
            rgb[i++] = 0;
            rgb[i++] = 0;
        }

        rgb += 1;
        width -= 1;
        height -= 1;

        for (; height--; bayer += bayerStep, rgb += rgbStep) {
            const uint8_t* bayerEnd = bayer + width;

            if (start_with_green) {
                rgb[-blue] = bayer[1];
                rgb[0]     = (bayer[0] + bayer[bayerStep + 1] + 1) >> 1;
                rgb[blue]  = bayer[bayerStep];
                bayer++;
                rgb += 3;
            }

            if (blue > 0) {
                for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                    rgb[-1] = bayer[0];
                    rgb[0]  = (bayer[1] + bayer[bayerStep] + 1) >> 1;
                    rgb[1]  = bayer[bayerStep + 1];

                    rgb[2] = bayer[2];
                    rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
                    rgb[4] = bayer[bayerStep + 1];
                }
            }
            else {
                for (; bayer <= bayerEnd - 2; bayer += 2, rgb += 6) {
                    rgb[1]  = bayer[0];
                    rgb[0]  = (bayer[1] + bayer[bayerStep] + 1) >> 1;
                    rgb[-1] = bayer[bayerStep + 1];

                    rgb[4] = bayer[2];
                    rgb[3] = (bayer[1] + bayer[bayerStep + 2] + 1) >> 1;
                    rgb[2] = bayer[bayerStep + 1];
                }
            }

            if (bayer < bayerEnd) {
                rgb[-blue] = bayer[0];
                rgb[0]     = (bayer[1] + bayer[bayerStep] + 1) >> 1;
                rgb[blue]  = bayer[bayerStep + 1];
                bayer++;
                rgb += 3;
            }

            bayer -= width;
            rgb -= width * 3;

            blue             = -blue;
            start_with_green = !start_with_green;
        }

        return output;
    }

    message::input::Image debayer(const message::input::Image& image, DEBAYER_METHOD method) {

        message::input::Image rgb_image;
        // Convert from bayer to RGB
        std::vector<uint8_t> rgb_data(image.dimensions.x() * image.dimensions.y());
        switch (method) {
            case DEBAYER_METHOD::SIMPLE:
                rgb_data = debayer_simple(image.data.data(), image.dimensions.x(), image.dimensions.y(), image.format);
                break;
            case DEBAYER_METHOD::BILINEAR:
                rgb_data =
                    debayer_bilinear(image.data.data(), image.dimensions.x(), image.dimensions.y(), image.format);
                break;
            case DEBAYER_METHOD::HQLINEAR:
                rgb_data =
                    debayer_hqlinear(image.data.data(), image.dimensions.x(), image.dimensions.y(), image.format);
                break;
            case DEBAYER_METHOD::EDGESENSE:
                rgb_data =
                    debayer_edgesense(image.data.data(), image.dimensions.x(), image.dimensions.y(), image.format);
                break;
            default: throw new std::runtime_error("Invalid demosaicing method");
        }

        // Copy across everyting in image to out image except format and data
        rgb_image.dimensions = image.dimensions;
        rgb_image.camera_id  = image.camera_id;
        rgb_image.name       = image.name;
        rgb_image.timestamp  = image.timestamp;
        rgb_image.Hcw        = image.Hcw;
        rgb_image.lens       = image.lens;
        // Set output image to converted RGB image
        rgb_image.format = utility::vision::fourcc("RGB8");
        rgb_image.data   = rgb_data;

        return rgb_image;
    }

}  // namespace output
}  // namespace module
