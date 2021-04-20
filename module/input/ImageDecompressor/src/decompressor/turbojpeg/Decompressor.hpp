#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_DECOMPRESSOR_HPP
#define MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_DECOMPRESSOR_HPP
#include "../Decompressor.hpp"

#include "utility/vision/mosaic.hpp"

namespace module::input::decompressor::turbojpeg {

    class Decompressor : public decompressor::Decompressor {
    public:
        Decompressor(const uint32_t& width, const uint32_t& height, const uint32_t& format);
        Decompressor(const Decompressor&) = default;
        Decompressor(Decompressor&&)      = default;
        Decompressor& operator=(const Decompressor&) = default;
        Decompressor& operator=(Decompressor&&) = default;
        virtual ~Decompressor();
        std::pair<std::vector<uint8_t>, int> decompress(const std::vector<uint8_t>& data) override;

    private:
        /// Either a mosaic permutation table if this is a mosaic pattern, or an empty list
        utility::vision::Mosaic mosaic;

        /// The fourcc code we output
        uint32_t output_fourcc;
    };

}  // namespace module::input::decompressor::turbojpeg

#endif  // MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_TURBOJPEG_DECOMPRESSOR_HPP
