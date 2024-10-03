/*
 * MIT License
 *
 * Copyright (c) 2019 NUbots
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
#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_HPP
#define MODULE_OUTPUT_IMAGECOMPRESSOR_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <nuclear>

#include "compressor/CompressorFactory.hpp"

namespace module::output {

    class ImageCompressor : public NUClear::Reactor {
    private:
        struct CompressorContext {
            struct Compressor {
                /// Mutex controlling access to this compressor instance
                std::unique_ptr<std::mutex> mutex;
                /// The actual compressor that will convert the image to jpeg format
                std::shared_ptr<compressor::Compressor> compressor;
            };

            /// A list of compressors that can be used
            std::vector<Compressor> compressors;

            /// The width height and format so that if these change we can regenerate the compressors
            uint32_t width{};
            uint32_t height{};
            uint32_t format{};
        };

    public:
        /// @brief Called by the powerplant to build and setup the ImageCompressor reactor.
        explicit ImageCompressor(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            /// The compressor factories that will be used to create a compression context
            std::vector<std::pair<std::shared_ptr<compressor::CompressorFactory>, int>> factories;
        } config;

        /// The compressors that are available for use
        std::mutex compressor_mutex;
        std::map<uint32_t, std::shared_ptr<CompressorContext>> compressors;

        /// Number of images that have been compressed since this was last reset
        int compressed = 0;
        /// Number of images that have been dropped since this was last reset
        int dropped = 0;
    };

}  // namespace module::output

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_HPP
