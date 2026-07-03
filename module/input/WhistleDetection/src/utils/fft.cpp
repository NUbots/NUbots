#include "fft.hpp"

#include <cmath>
#include <cstdlib>

namespace module::input {

    void fft(std::vector<std::complex<float>>& x) {
        const size_t N = x.size();

        // Bit-reversal permutation
        for (size_t i = 1, j = 0; i < N; i++) {
            size_t bit = N >> 1;
            for (; j & bit; bit >>= 1) {
                j ^= bit;
            }
            j ^= bit;
            if (i < j) {
                std::swap(x[i], x[j]);
            }
        }

        // Butterfly stages
        for (size_t len = 2; len <= N; len <<= 1) {
            const float ang = -2.0f * static_cast<float>(M_PI) / static_cast<float>(len);
            const std::complex<float> wlen(std::cos(ang), std::sin(ang));
            for (size_t i = 0; i < N; i += len) {
                std::complex<float> w(1.0f, 0.0f);
                for (size_t j = 0; j < len / 2; j++) {
                    const auto u       = x[i + j];
                    const auto v       = x[i + j + len / 2] * w;
                    x[i + j]           = u + v;
                    x[i + j + len / 2] = u - v;
                    w *= wlen;
                }
            }
        }
    }

}  // namespace module::input
