/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP
#define MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP

#include <cstdint>
#include <memory>

#include "Compressor.hpp"

namespace module::output::compressor {

    class CompressorFactory {
    public:
        CompressorFactory()                                        = default;
        virtual ~CompressorFactory()                               = default;
        CompressorFactory(const CompressorFactory&)                = default;
        CompressorFactory(CompressorFactory&&) noexcept            = default;
        CompressorFactory& operator=(const CompressorFactory&)     = default;
        CompressorFactory& operator=(CompressorFactory&&) noexcept = default;

        virtual std::shared_ptr<Compressor> make_compressor(const uint32_t& width,
                                                            const uint32_t& height,
                                                            const uint32_t& format) = 0;
    };

}  // namespace module::output::compressor

#endif  // MODULE_OUTPUT_COMPRESSOR_COMPRESSORFACTORY_HPP
