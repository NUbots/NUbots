#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_TURBOJPEG_COMPRESSOR_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_TURBOJPEG_COMPRESSOR_H

#include "../Compressor.h"

namespace module {
namespace output {
    namespace compressor {
        namespace turbojpeg {

            class Compressor : public compressor::Compressor {
            public:
                Compressor(const int& quality, const uint32_t& width, const uint32_t& height, const uint32_t& format);
                virtual ~Compressor();
                virtual std::vector<uint8_t> compress(const std::vector<uint8_t>& data,
                                                      const uint32_t& width,
                                                      const uint32_t& height,
                                                      const uint32_t& format) override;

            private:
                /// The JPEG quality to compress with
                int quality;
                /// Either a mosaic permutation table if this is a mosaic pattern, or an empty list
                std::vector<uint32_t> mosaic_table;
            };

        }  // namespace turbojpeg
    }      // namespace compressor
}  // namespace output
}  // namespace module

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_TURBOJPEG_COMPRESSOR_H
