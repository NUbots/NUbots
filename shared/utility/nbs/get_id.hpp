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
#ifndef UTILITY_NBS_GET_ID_HPP
#define UTILITY_NBS_GET_ID_HPP

#include <type_traits>

#include "utility/type_traits/has_id.hpp"

namespace utility::nbs {

    /// @brief Returns the subtype field of data or, if subtype does not exist, 0
    template <typename T>
    std::enable_if_t<!utility::type_traits::has_id<T>::value, uint32_t> get_subtype(const T& /*data*/) {
        return 0;
    }

    template <typename T>
    std::enable_if_t<utility::type_traits::has_id<T>::value, uint32_t> get_subtype(const T& data) {
        return data.id;
    }

    /// @brief Returns the id field of data or, if id does not exist, 0
    template <typename T>
    std::enable_if_t<!utility::type_traits::has_id<T>::value, uint32_t> get_id(const T& /*data*/) {
        return 0;
    }

    template <typename T>
    std::enable_if_t<utility::type_traits::has_id<T>::value, uint32_t> get_id(const T& data) {
        return data.id;
    }

}  // namespace utility::nbs

#endif  // UTILITY_NBS_GET_ID_HPP
