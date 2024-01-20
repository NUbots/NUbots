/*
 * MIT License
 *
 * Copyright (c) 2013 NUbots
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

#ifndef NUHELPERS_HPP
#define NUHELPERS_HPP

#include <nuclear>
#include <utility>

#include "message/eye/DataPoint.hpp"

#include "utility/type_traits/is_iterable.hpp"

namespace utility::nusight {

    using message::eye::DataPoint;

    namespace helpers {
        using message::eye::DataPoint;
        using utility::type_traits::is_iterable;

        inline void buildGraph(DataPoint& /*dataPoint*/) {}

        template <typename First, typename... Remainder>
        typename std::enable_if<!is_iterable<First>::value>::type buildGraph(DataPoint& dataPoint,
                                                                             First first,
                                                                             Remainder... remainder) {
            dataPoint.value.push_back(first);
            buildGraph(dataPoint, remainder...);
        }

        template <typename First, typename... Remainder>
        typename std::enable_if<is_iterable<First>::value>::type buildGraph(DataPoint& dataPoint,
                                                                            First first,
                                                                            Remainder... remainder) {
            for (const auto& value : first) {
                dataPoint.value.push_back(value);
            }
            buildGraph(dataPoint, remainder...);
        }
    }  // namespace helpers

    template <typename... Values>
    inline std::unique_ptr<message::eye::DataPoint> graph(std::string label, Values... values) {
        auto dataPoint   = std::make_unique<DataPoint>();
        dataPoint->label = std::move(label);
        helpers::buildGraph(*dataPoint, values...);
        return dataPoint;
    }

}  // namespace utility::nusight
#endif
