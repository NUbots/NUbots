#ifndef MODULE_OUTPUT_COMPRESSOR_COMPRESSOR_HPP
#define MODULE_OUTPUT_COMPRESSOR_COMPRESSOR_HPP

#include <cstdint>
#include <vector>

namespace module::output::compressor {

    class Compressor {
    public:
        virtual std::vector<uint8_t> compress(const std::vector<uint8_t>& data) = 0;
    };

}  // namespace module::output::compressor


#endif  // MODULE_OUTPUT_COMPRESSOR_COMPRESSOR_HPP
