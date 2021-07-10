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

#include <catch.hpp>
#include <functional>
#include <nuclear>

#include "EmissionCatcher.hpp"

namespace extension::moduletest {

    template <typename Module>
    class ModuleTest {

        // TODO: Sort out Configuration of the module
        explicit ModuleTest(std::string module_name)
            : module_being_tested(std::make_unique<NUClear::Environment>(powerplant, module_name, NUClear::FATAL)) {}

        [[nodiscard]] Module& setup() {
            return module_being_tested;
        }

        // TODO return the resulting emissions SOMEHOW??
        template <typename TriggerMessage>
        void run(NUClear::threading::ReactionHandle& handle_to_trigger, TriggerMessage& trigger) {
            handle_to_trigger.enable();
            // Call handle's task function with trigger as argument
            // Catch the response emits (somehow) using TestPowerPlant's overridden `emit`
            // Return the response emits (somehow)
        }

    private:
        TestPowerPlant powerplant{};
        Module module_being_tested;
    }

}  // namespace extension::moduletest

#endif  // EXTENSION_MODULETEST_HPP
