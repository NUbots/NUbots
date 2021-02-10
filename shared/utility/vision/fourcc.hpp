#ifndef UTILITY_VISION_FOURCC
#define UTILITY_VISION_FOURCC

#include <cstdint>
#include <string>

namespace utility {
namespace vision {

    inline constexpr uint32_t fourcc(const char (&c)[5]) {
        return c[3] << 24 | c[2] << 16 | c[1] << 8 | c[0];
    }

    inline std::string fourcc(const uint32_t& v) {
        const char ch[5] = {static_cast<char>((v >> 0) & 0xFF),
                            static_cast<char>((v >> 8) & 0xFF),
                            static_cast<char>((v >> 16) & 0xFF),
                            static_cast<char>((v >> 24) & 0xFF),
                            0};
        return ch;
    }

}  // namespace vision
}  // namespace utility

#endif  // UTILITY_VISION_FOURCC
