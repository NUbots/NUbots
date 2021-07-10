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

#ifndef EXTENSION_MODULETEST_TESTPOWERPLANT_HPP
#define EXTENSION_MODULETEST_TESTPOWERPLANT_HPP

#include <catch.hpp>
#include <functional>
#include <memory>
#include <nuclear>
#include <vector>

namespace extension::moduletest {

    template <typename Module>
    class TestPowerPlant : public NUClear::PowerPlant {

        TestPowerPlant() = default;

        template <typename T>
        void emit(std::unique_ptr<T>&& data) override {
            // TODO (somehow) return this data to the tester
        }
    };
}  // namespace extension::moduletest

#endif  // EXTENSION_MODULETEST_TEST_POWERPLANT_HPP
