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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#include <catch.hpp>
#include <type_traits>

#include "utility/math/Transform.hpp"

using utility::math::Space;
using utility::math::Transform;

namespace {
    template <typename Scalar, Space LTo, Space LFrom, Space RTo, Space RFrom>
    [[nodiscard]] bool spaces_are_compatible(Transform<Scalar, LTo, LFrom> /*lTransform*/,
                                             Transform<Scalar, RTo, RFrom> /*rTransform*/) {
        return LFrom == RTo;
    }

    template <typename Scalar, Space ActualTo, Space ActualFrom, Space RequiredTo>
    [[nodiscard]] bool to_space_correct(Transform<Scalar, ActualTo, ActualFrom> /*transform*/) {
        return ActualTo == RequiredTo;
    }

    template <typename Scalar, Space ActualTo, Space ActualFrom, Space RequiredFrom>
    [[nodiscard]] bool from_space_correct(Transform<Scalar, ActualTo, ActualFrom> /*transform*/) {
        return ActualFrom == RequiredFrom;
    }
}  // namespace


SCENARIO("Transforms can be multiplied") {

    GIVEN("Two transforms with compatible spaces") {

        auto Htc = Transform<double, Space::TORSO, Space::CAMERA>();
        auto Hcf = Transform<double, Space::CAMERA, Space::FIELD>();

        // Validate our GIVEN clause assumption
        THEN("Spaces are compatible") {
            REQUIRE(spaces_are_compatible(Htc, Hcf));
        }

        WHEN("The Transforms are multiplied") {
            Transform<double, Space::TORSO, Space::FIELD> Htf = Htc * Hcf;

            THEN("The result has the correct spaces") {
                REQUIRE(from_space_correct<double, Space::TORSO, Space::FIELD, Space::FIELD>(Htf));
                REQUIRE(to_space_correct<double, Space::TORSO, Space::FIELD, Space::TORSO>(Htf));
            }
        }
    }
}
