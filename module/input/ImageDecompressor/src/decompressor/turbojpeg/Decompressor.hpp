#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_DECOMPRESSOR_HPP
#define MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_DECOMPRESSOR_HPP
#include "../Decompressor.hpp"

#include "utility/vision/mosaic.hpp"

namespace module::input::decompressor::turbojpeg {

class Decompressor : public decompressor::Decompressor {
public:
    Decompressor(const uint32_t& width, const uint32_t& height, const uint32_t& format);
    virtual ~Decompressor();
    virtual std::pair<std::vector<uint8_t>, int> decompress(const std::vector<uint8_t>& data) override;

private:
    /// Either a mosaic permutation table if this is a mosaic pattern, or an empty list
    utility::vision::Mosaic mosaic;

    /// Expected width for decoding
    int width;
    /// Expected height for decoding
    int height;
    /// Expected input format for decoding
    int format;

    /// The fourcc code we output
    uint32_t output_fourcc;
};

}  // namespace module::input::decompressor::turbojpeg

#endif  // MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_DECOMPRESSOR_HPP
