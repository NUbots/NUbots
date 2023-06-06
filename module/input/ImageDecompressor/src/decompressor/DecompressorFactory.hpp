#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSORFACTORY_HPP
#define MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSORFACTORY_HPP

#include <cstdint>
#include <memory>

#include "Decompressor.hpp"

namespace module::input::decompressor {

    class DecompressorFactory {
    public:
        DecompressorFactory()                               = default;
        virtual ~DecompressorFactory()                      = default;
        DecompressorFactory(const DecompressorFactory&)     = default;
        DecompressorFactory(DecompressorFactory&&) noexcept = default;
        DecompressorFactory& operator=(const DecompressorFactory&) = default;
        DecompressorFactory& operator=(DecompressorFactory&&) noexcept = default;

        virtual std::shared_ptr<Decompressor> make_decompressor(const uint32_t& width,
                                                                const uint32_t& height,
                                                                const uint32_t& format) = 0;
    };

}  // namespace module::input::decompressor

#endif  // MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSORFACTORY_HPP
