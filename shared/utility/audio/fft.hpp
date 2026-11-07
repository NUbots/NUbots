#ifndef MODULE_INPUT_WHISTLEDETECTION_FFT_HPP
#define MODULE_INPUT_WHISTLEDETECTION_FFT_HPP

#include <complex>
#include <vector>

namespace module::input {

    /// Radix-2 Cooley-Tukey FFT — in-place, N must be a power of two.
    void fft(std::vector<std::complex<float>>& x);

}  // namespace module::input

#endif  // MODULE_INPUT_WHISTLEDETECTION_FFT_HPP
