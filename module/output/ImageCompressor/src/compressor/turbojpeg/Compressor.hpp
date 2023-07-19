#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_TURBOJPEG_COMPRESSOR_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_TURBOJPEG_COMPRESSOR_HPP

#include "../Compressor.hpp"

#include "utility/vision/mosaic.hpp"

namespace module::output::compressor::turbojpeg {

    class Compressor : public compressor::Compressor {
    public:
        Compressor(const int& quality, const uint32_t& width, const uint32_t& height, const uint32_t& format);
        std::vector<uint8_t> compress(const std::vector<uint8_t>& data) override;

    private:
        /// The JPEG quality to compress with
        int quality;
        /// The image width
        uint32_t width;
        /// The image height
        uint32_t height;
        /// The image format
        uint32_t format;
        /// The mosaic permutation table if this is a mosaic pattern
        utility::vision::Mosaic mosaic;
    };

}  // namespace module::output::compressor::turbojpeg

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_TURBOJPEG_COMPRESSOR_HPP
