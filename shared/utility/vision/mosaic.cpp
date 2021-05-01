#include "mosaic.hpp"

namespace utility::vision {

    Mosaic::Mosaic() = default;

    Mosaic::Mosaic(const uint32_t& width, const uint32_t& height, const uint32_t& format) {

        // Reserve the correct number of elements
        table.reserve(width * height);

        // Loop through all the coordinates and calculate the new coordinates
        int factor = Mosaic::size(format);
        for (uint32_t y = 0; y < height; ++y) {
            for (uint32_t x = 0; x < width; ++x) {
                // Local x and y are the same for all within the mosaic block
                uint32_t new_x = (x % factor) * (width / factor) + x / factor;
                uint32_t new_y = (y % factor) * (height / factor) + y / factor;

                table.push_back(new_y * width + new_x);
            }
        }
    }

    void Mosaic::permute(const uint8_t* src, uint8_t* dst) const {
        for (uint32_t i = 0; i < table.size(); ++i) {
            dst[table[i]] = src[i];
        }
    }

    void Mosaic::unpermute(const uint8_t* src, uint8_t* dst) const {
        for (uint32_t i = 0; i < table.size(); ++i) {
            dst[i] = src[table[i]];
        }
    }

    std::vector<uint8_t> Mosaic::permute(const std::vector<uint8_t>& data) const {
        // Permute our bytes into a new order
        std::vector<uint8_t> permuted(data.size());
        permute(data.data(), permuted.data());
        return permuted;
    }

    std::vector<uint8_t> Mosaic::unpermute(const std::vector<uint8_t>& data) const {
        // Permute our bytes into a new order
        std::vector<uint8_t> unpermuted(data.size());
        unpermute(data.data(), unpermuted.data());
        return unpermuted;
    }

    Mosaic::operator bool() const {
        return !table.empty();
    }

    int Mosaic::size(const uint32_t& format) {
        switch (format) {
            case utility::vision::fourcc("BGGR"):  // Colour Bayer
            case utility::vision::fourcc("RGGB"):  // Colour Bayer
            case utility::vision::fourcc("GRBG"):  // Colour Bayer
            case utility::vision::fourcc("GBRG"):  // Colour Bayer
            case utility::vision::fourcc("JPBG"):  // Compressed Permuted Bayer
            case utility::vision::fourcc("JPRG"):  // Compressed Permuted Bayer
            case utility::vision::fourcc("JPGR"):  // Compressed Permuted Bayer
            case utility::vision::fourcc("JPGB"):  // Compressed Permuted Bayer
            case utility::vision::fourcc("PY8 "):  // Polarized Monochrome
                return 2;                          // Two values per width/height

            case utility::vision::fourcc("PBG8"):  // Polarized Colour Bayer
            case utility::vision::fourcc("PRG8"):  // Polarized Colour Bayer
            case utility::vision::fourcc("PGR8"):  // Polarized Colour Bayer
            case utility::vision::fourcc("PGB8"):  // Polarized Colour Bayer
                return 4;                          // Four values per width/height

            default: return 1;  // Default is no mosaic
        }
    }

}  // namespace utility::vision
