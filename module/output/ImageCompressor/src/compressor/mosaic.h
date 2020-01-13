#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_MOSAIC_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_MOSAIC_H

#include <cstdint>

#include "compressor/CompressorFactory.h"
#include "utility/vision/fourcc.h"

namespace module::output::compressor::mosaic {

inline int mosaic_size(const uint32_t& format) {
    switch (format) {
        case utility::vision::fourcc("BGGR"):  // Colour Bayer
        case utility::vision::fourcc("RGGB"):  // Colour Bayer
        case utility::vision::fourcc("GRBG"):  // Colour Bayer
        case utility::vision::fourcc("GBRG"):  // Colour Bayer
        case utility::vision::fourcc("PY8 "):  // Polarized Monochrome
            return 2;                          // Two values per width/height

        case utility::vision::fourcc("PBG8"):  // Polarized Colour Bayer
        case utility::vision::fourcc("PRG8"):  // Polarized Colour Bayer
        case utility::vision::fourcc("PGR8"):  // Polarized Colour Bayer
        case utility::vision::fourcc("PGB8"):  // Polarized Colour Bayer
            return 4;                          // Four values per width/height

        default: return 0;
    }
}

inline std::vector<uint32_t> build_table(const uint32_t& width, const uint32_t& height, const uint32_t& format) {

    // Make an output vector with the right number of elements
    std::vector<uint32_t> output;
    output.reserve(width * height);

    // Loop through all the coordinates and calculate the new coordinates
    int factor = mosaic_size(format);
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            // Local x and y are the same for all within the mosaic block
            uint32_t new_x = (x % factor) * (width / factor) + x / factor;
            uint32_t new_y = (y % factor) * (height / factor) + y / factor;

            output.push_back(new_y * width + new_x);
        }
    }

    return output;
}

}  // namespace module::output::compressor::mosaic

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_MOSAIC_H
