#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSOR_HPP
#define MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSOR_HPP

#include <cstdint>
#include <vector>

namespace module::input::decompressor {

    class Decompressor {
    public:
        Decompressor()                        = default;
        virtual ~Decompressor()               = default;
        Decompressor(const Decompressor&)     = default;
        Decompressor(Decompressor&&) noexcept = default;
        Decompressor& operator=(const Decompressor&) = default;
        Decompressor& operator=(Decompressor&&) noexcept = default;

        virtual std::pair<std::vector<uint8_t>, int> decompress(const std::vector<uint8_t>& data) = 0;
    };

}  // namespace module::input::decompressor


#endif  // MODULE_INPUT_IMAGEDECOMPRESSOR_DECOMPRESSOR_DECOMPRESSOR_HPP
