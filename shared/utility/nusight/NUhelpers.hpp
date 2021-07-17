/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef NUHELPERS_HPP
#define NUHELPERS_HPP

#include <nuclear>
#include <utility>

#include "message/support/nusight/DataPoint.hpp"

#include "utility/type_traits/is_iterable.hpp"

namespace utility::nusight {

    using message::support::nusight::DataPoint;

    namespace helpers {
        using message::support::nusight::DataPoint;
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
    inline std::unique_ptr<message::support::nusight::DataPoint> graph(std::string label, Values... values) {
        auto dataPoint   = std::make_unique<DataPoint>();
        dataPoint->label = std::move(label);
        helpers::buildGraph(*dataPoint, values...);
        return dataPoint;
    }

}  // namespace utility::nusight
#endif
