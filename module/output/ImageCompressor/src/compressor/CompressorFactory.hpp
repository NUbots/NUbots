#ifndef MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP
#define MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP

#include <cstdint>
#include <memory>

#include "Compressor.hpp"

namespace module::output::compressor {

    class CompressorFactory {
    public:
        virtual std::shared_ptr<Compressor> make_compressor(const uint32_t& width,
                                                            const uint32_t& height,
                                                            const uint32_t& format) = 0;
    };

}  // namespace module::output::compressor

#endif  // MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP
