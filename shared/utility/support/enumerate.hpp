/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#ifndef UTILITY_SUPPORT_ENUMERATE
#define UTILITY_SUPPORT_ENUMERATE

#include <limits>
#include <ranges>
#include <utility>

namespace utility::support {

    /**
     * @brief Allows for python like enumeration
     * @warning Will cause problems if iterable is destroyed
     * @tparam T The type of the iterable to iterate over, should be automatically deduced
     */
    template <std::ranges::range T>
    struct enumerate {
    private:
        T& iterable;

    public:
        /**
         *  @brief A wrapper for the actual iterator
         *  @warning Is invalidated like an iterator for T
         */
        struct iterator {
            using iterator_type = std::ranges::iterator_t<T>;
            int i{0};
            iterator_type iter{};

            /// @brief Compares the actual iterators
            bool operator!=(const iterator& other) const {
                return iter != other.iter;
            }

            /// @brief Increments the actual iterator and the counter
            void operator++() {
                i++;
                iter++;
            }

            /**
             * @brief Gets the count and dereferences the actual iterator
             * @return first is count, second is dereferenced value
             */
            auto operator*() {
                return std::pair(i, *iter);
            }

            iterator(const int& i, iterator_type&& iter) : i(i), iter(std::forward<iterator_type>(iter)) {}
        };


        /// @brief An iterator to the start of the iterable
        constexpr iterator begin() noexcept {
            return iterator(0, std::ranges::begin(iterable));
        }

        /// @brief An iterator to the end of the iterable
        constexpr iterator end() noexcept {
            return iterator(std::numeric_limits<int>::max(), std::ranges::end(iterable));
        }

        /**
         * @brief Wraps an iterable to behave like python enumerate
         * @param iterable An iterable container to wrap
         */
        enumerate(T& iterable) : iterable(iterable) {}
    };
}  // namespace utility::support


#endif  // UTILITY_SUPPORT_ENUMERATE
