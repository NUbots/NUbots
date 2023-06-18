#ifndef MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP
#define MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP

#include <cstdint>
#include <memory>

#include "Compressor.hpp"

namespace module::output::compressor {

    class CompressorFactory {
    public:
        CompressorFactory()                             = default;
        virtual ~CompressorFactory()                    = default;
        CompressorFactory(const CompressorFactory&)     = default;
        CompressorFactory(CompressorFactory&&) noexcept = default;
        CompressorFactory& operator=(const CompressorFactory&) = default;
        CompressorFactory& operator=(CompressorFactory&&) noexcept = default;

        virtual std::shared_ptr<Compressor> make_compressor(const uint32_t& width,
                                                            const uint32_t& height,
                                                            const uint32_t& format) = 0;
    };

}  // namespace module::output::compressor

#endif  // MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP
