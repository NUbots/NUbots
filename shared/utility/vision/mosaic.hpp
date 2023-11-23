/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#ifndef UTILITY_VISION_MOSAIC_HPP
#define UTILITY_VISION_MOSAIC_HPP

#include <cstdint>
#include <vector>

#include "utility/vision/fourcc.hpp"

namespace utility::vision {

    /**
     * This class is used to convert images from a permuted mosaic form to a regular mosaic form.
     * The use case for this is to make bayer mosaic patterns more compressable for format like JPEG by removing the
     * hard edges that occur on colour transitions in a mosaiced image. For example it would be able to perform the
     * conversion between the following image permutations.
     *
     * RGRGRGRGRGRGRGRGRGRG          RRRRRRRRRRGGGGGGGGGG
     * GBGBGBGBGBGBGBGBGBGB          RRRRRRRRRRGGGGGGGGGG
     * RGRGRGRGRGRGRGRGRGRG          RRRRRRRRRRGGGGGGGGGG
     * GBGBGBGBGBGBGBGBGBGB          RRRRRRRRRRGGGGGGGGGG
     * RGRGRGRGRGRGRGRGRGRG    ->    RRRRRRRRRRGGGGGGGGGG
     * GBGBGBGBGBGBGBGBGBGB          GGGGGGGGGGBBBBBBBBBB
     * RGRGRGRGRGRGRGRGRGRG          GGGGGGGGGGBBBBBBBBBB
     * GBGBGBGBGBGBGBGBGBGB          GGGGGGGGGGBBBBBBBBBB
     * RGRGRGRGRGRGRGRGRGRG          GGGGGGGGGGBBBBBBBBBB
     * GBGBGBGBGBGBGBGBGBGB          GGGGGGGGGGBBBBBBBBBB
     *
     * Each of the four components in the bayer are spread out into a distinct corner of the image.
     *
     */
    class Mosaic {
    public:
        Mosaic();
        Mosaic(const uint32_t& width, const uint32_t& height, const uint32_t& format);

        /**
         * @brief Permute the data from its standard mosaic into a separated mosaic
         * Note that the destination buffer must be the same size as the input buffer.
         * Both of these buffers must be width * height that was set when creating this mosaic instance.
         *
         * @param src   the pointer to the source buffer to permute
         * @param dst   the pointer to the destination buffer to permute
         * @param size  the number of bytes in the source and destination buffers
         */
        void permute(const uint8_t* src, uint8_t* dst) const;

        /**
         * @brief Unpermute the data from its separated mosaic into a standard mosaic
         * Note that the destination buffer must be the same size as the input buffer.
         * Both of these buffers must be width * height that was set when creating this mosaic instance.
         *
         * @param src   the pointer to the source buffer to unpermute
         * @param dst   the pointer to the destination buffer to unpermute
         * @param size  the number of bytes in the source and destination buffers
         */
        void unpermute(const uint8_t* src, uint8_t* dst) const;

        /**
         * @brief Permute the data from its standard mosaic into a separated mosaic
         *
         * @param data the input data to permute
         * @return a new std::vector object containing the permuted image
         */
        [[nodiscard]] std::vector<uint8_t> permute(const std::vector<uint8_t>& data) const;

        /**
         * @brief Unpermute the data from its separated mosaic into a standard mosaic
         *
         * @param data the input data to unpermute
         * @return a new std::vector object containing the unpermuted image
         */
        [[nodiscard]] std::vector<uint8_t> unpermute(const std::vector<uint8_t>& data) const;

        /**
         * @brief Gets the mosaic size of a type from its fourcc code
         *
         * @param format the fourcc type that we are getting the mosaic size for
         *
         * @return the dimensionality of the mosaic size (e.g. 1 for no mosaic, 2 for bayer, 4 for polarized bayer)
         */
        static int size(const uint32_t& format);

        /**
         * @brief Returns true if this mosaic has been initialised with a type that requires demosaicking
         *
         * @return true if the mosaic instance is valid
         * @return false if the mosaic instance is invalid
         */
        operator bool() const;

    private:
        /// The permutation table used to rearrange the data
        std::vector<uint32_t> table;
    };

}  // namespace utility::vision

#endif  // UTILITY_VISION_MOSAIC_HPP
