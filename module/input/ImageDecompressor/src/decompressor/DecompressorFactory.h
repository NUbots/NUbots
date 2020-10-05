#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSORFACTORY_H
#define MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSORFACTORY_H

#include <cstdint>
#include <memory>

#include "Decompressor.h"

namespace module::input::decompressor {

class DecompressorFactory {
public:
    virtual std::shared_ptr<Decompressor> make_decompressor(const uint32_t& width,
                                                            const uint32_t& height,
                                                            const uint32_t& format) = 0;
};

}  // namespace module::input::decompressor

#endif  // MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSORFACTORY_H
