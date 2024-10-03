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
#ifndef UTILITY_TYPE_TRAITS_IS_ITERABLE
#define UTILITY_TYPE_TRAITS_IS_ITERABLE

#include <utility>

namespace utility::type_traits {

    /**
     * @brief SFINAE struct to test if the passed class has a begin and an end.
     *
     * @tparam T the class to check
     *
     * @note Replaced by range concepts in c++20
     */
    template <typename T>
    struct is_iterable {
    private:
        using yes = std::true_type;
        using no  = std::false_type;

        template <typename U>
        static auto test_begin(int) -> decltype(std::declval<U>().begin(), yes());
        template <typename>
        static no test_begin(...);

        template <typename U>
        static auto test_end(int) -> decltype(std::declval<U>().end(), yes());
        template <typename>
        static no test_end(...);

    public:
        static constexpr bool value =
            std::is_same<decltype(test_begin<T>(0)), yes>::value && std::is_same<decltype(test_end<T>(0)), yes>::value;
    };
}  // namespace utility::type_traits

#endif  // UTILITY_TYPE_TRAITS_IS_ITERABLE
