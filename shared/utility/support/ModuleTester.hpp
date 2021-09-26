/*
 * This file is part of NUbots Codebase.
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

#ifndef UTILITY_SUPPORT_MODULETESTER
#define UTILITY_SUPPORT_MODULETESTER

#include <catch.hpp>

#include "TestLogHandler.hpp"

namespace utility::support {

    template <typename ModuleToTest>
    class [[nodiscard]] ModuleTester {
    public:
        ModuleTester(const int& num_threads = 1) : plant(gen_config(num_threads)) {

            INFO("Installing TestLogHandler (automatic for all module tests)");
            plant.install<utility::support::TestLogHandler>();
        }

        template <typename Module>
        void install(const std::string& module_name) {
            INFO("Installing " << module_name);
            plant.install<Module>();
        }

        void run() {
            INFO("Installing the module being tested");
            plant.install<ModuleToTest>();
            INFO("Starting PowerPlant");
            plant.start();
        }

    private:
        NUClear::PowerPlant plant;

        NUClear::PowerPlant::Configuration gen_config(const int& num_threads) {
            NUClear::PowerPlant::Configuration config;
            config.thread_count = num_threads;
            return config;
        }
    };

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_MODULETESTER
