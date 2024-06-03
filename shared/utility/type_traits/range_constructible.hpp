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
#ifndef UTILITY_TYPE_TRAITS_RANGE_CONSTRUCTIBLE
#define UTILITY_TYPE_TRAITS_RANGE_CONSTRUCTIBLE

#include <concepts>
#include <ranges>
#include <type_traits>
#include <utility>

namespace utility::type_traits {

    /**
     * @brief Checks that range has both, a begin method and an end method, and that the begin method returns something
     * that can be dereferenced to a type implicitly convertible to vale_type
     * @tparam range The type that should be checked
     * @tparam value_type The type to be implicit converted to
     */
    template <typename range, typename value_type>
    concept range_constructible =
        std::ranges::input_range<range> && std::convertible_to<decltype(*std::declval<range>().begin()),
                                                               std::remove_cvref_t<value_type>>;

}  // namespace utility::type_traits

#endif  // UTILITY_TYPE_TRAITS_RANGE_CONSTRUCTIBLE