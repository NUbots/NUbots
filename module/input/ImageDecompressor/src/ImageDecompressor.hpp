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
#ifndef MODULE_INPUT_IMAGEDECOMPRESSOR_HPP
#define MODULE_INPUT_IMAGEDECOMPRESSOR_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <nuclear>

#include "decompressor/DecompressorFactory.hpp"

namespace module::input {

    class ImageDecompressor : public NUClear::Reactor {
    private:
        struct DecompressorContext {
            struct Decompressor {
                /// Mutex controlling access to this decompressor instance
                std::unique_ptr<std::mutex> mutex;
                /// The actual decompressor that will convert the image to jpeg format
                std::shared_ptr<decompressor::Decompressor> decompressor;
            };

            /// A list of decompressors that can be used
            std::vector<Decompressor> decompressors;

            /// The format so that if these change we can regenerate the decompressors
            uint32_t width{};
            uint32_t height{};
            uint32_t format{};
        };

    public:
        /// @brief Called by the powerplant to build and setup the ImageDecompressor reactor.
        explicit ImageDecompressor(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct {
            /// The decompressor factories that will be used to create a decompression context
            std::vector<std::pair<std::shared_ptr<decompressor::DecompressorFactory>, int>> factories;
        } config;

        /// The decompressors that are available for use
        std::mutex decompressor_mutex;
        std::map<uint32_t, std::shared_ptr<DecompressorContext>> decompressors;

        /// Number of images that have been decompressed since this was last reset
        int decompressed = 0;
        /// Number of images that have been dropped since this was last reset
        int dropped = 0;
    };

}  // namespace module::input

#endif  // MODULE_INPUT_IMAGEDECOMPRESSOR_HPP
