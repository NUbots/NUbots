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

#ifndef UTILITY_MATH_RANSAC_RANSACRESULT_HPP
#define UTILITY_MATH_RANSAC_RANSACRESULT_HPP

#include <nuclear>
#include <utility>
#include <vector>

namespace utility::math::ransac {

    template <typename Iterator, typename Model>
    struct RansacResult {
    public:
        RansacResult() : model(), first(), last() {}

        RansacResult(const Model& model, const Iterator& first, const Iterator& last)
            : model(model), first(first), last(last) {}

        Model model;

        Iterator begin() const {
            return first;
        }

        Iterator end() const {
            return last;
        }

    private:
        Iterator first;
        Iterator last;
    };
}  // namespace utility::math::ransac
#endif
