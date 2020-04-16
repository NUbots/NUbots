#include "Factory.h"

namespace module {
namespace output {
    namespace compressor {
        namespace turbojpeg {

            Factory::Factory(const int& quality) : quality(quality) {}
            Factory::~Factory() {}

            std::shared_ptr<compressor::Compressor> Factory::make_compressor(const uint32_t width,
                                                                             const uint32_t& height,
                                                                             const uint32_t& format) {
                return std::make_shared<turbojpeg::Compressor>(quality, width, height, format);
            }

        }  // namespace turbojpeg
    }      // namespace compressor
}  // namespace output
}  // namespace module
