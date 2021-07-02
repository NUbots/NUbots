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

#ifndef EXTENSION_MODULETEST_HPP
#define EXTENSION_MODULETEST_HPP

#include <nuclear>

namespace extension {

    template <typename Module>
    class ModuleTest : NUClear::PowerPlant {
    public:
        ModuleTest() = delete;

        explicit ModuleTest(Module m) : module(m), NUClear::PowerPlant(getSingleThreadConfig()) {}
        ~ModuleTest() {
            shutdown();
        }
        ModuleTest(ModuleTest& other)  = delete;
        ModuleTest(ModuleTest&& other) = delete;
        ModuleTest& operator=(ModuleTest& other) = delete;
        ModuleTest&& operator=(ModuleTest&& other) = delete;

    private:
        static const NUClear::PowerPlant::Configuration& getSingleThreadConfig() {
            NUClear::PowerPlant::Configuration cfg;
            cfg.thread_count = 1;
            return cfg;
        }

        Module module;
    };

}  // namespace extension

#endif  // EXTENSION_MODULETEST_HPP
