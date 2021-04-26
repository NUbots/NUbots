#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSOR_HPP
#define MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSOR_HPP

#include <cstdint>
#include <vector>

namespace module::input::decompressor {

    class Decompressor {
    public:
        virtual std::pair<std::vector<uint8_t>, int> decompress(const std::vector<uint8_t>& data) = 0;
    };

}  // namespace module::input::decompressor


#endif  // MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSOR_HPP
