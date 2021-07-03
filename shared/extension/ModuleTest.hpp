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
#include <nuclear>
#include <tuple>
#include <type_traits>

namespace extension {

    template <typename Module>
    class ModuleTest : private NUClear::PowerPlant {
    public:
        ModuleTest() = delete;

        // TODO: somehow disable the reactions which aren't on<Trigger<..>>, such as on<Every<..>> or on<Always>

        explicit ModuleTest(const bool start_powerplant_automatically = true)
            : NUClear::PowerPlant(get_single_thread_config()) {
            // This assert could be worth playing with
            // static_assert(std::isconvertible<Module*, NUClear::Reactor*>);
            install<Module>();
            if (start_powerplant_automatically) {
                startup_manually();
            }
        }
        ~ModuleTest() {
            started = false;
            shutdown();
        }
        // TODO(KipHamiltons): add varargs to this
        void startup_manually() {
            INFO("Starting up ModuleTest power plant.");
            started = true;
            start();
        }
        void shutdown_manually() {
            INFO("Shutting down ModuleTest power plant. Further use will require a restart.");
            started = false;
            shutdown();
        }

        template <typename EmitType, typename... ResponseEmissionTypes>
        [[nodiscard]] auto run_reaction(const EmitType& emission) {
            std::vector<NUClear::threading::ReactionHandle> response_catchers =
                create_response_catchers<ResponseEmissionTypes...>();
            for (auto& response_catcher : response_catchers) {
                response_catcher.enable();
            }
            powerplant.emit<EmitType>(std::move(std::make_unique(emission)));

            for (auto& response_catcher : response_catchers) {
                response_catcher.disable();
            }
            return std::tuple_cat
        }

        ModuleTest(ModuleTest& other)  = delete;
        ModuleTest(ModuleTest&& other) = delete;
        ModuleTest& operator=(ModuleTest& other) = delete;
        ModuleTest&& operator=(ModuleTest&& other) = delete;

    private:
        static const NUClear::PowerPlant::Configuration& get_single_thread_config() {
            NUClear::PowerPlant::Configuration cfg;
            cfg.thread_count = 1;
            return cfg;
        }

        bool started = false;
    };

}  // namespace extension

#endif  // EXTENSION_MODULETEST_HPP
