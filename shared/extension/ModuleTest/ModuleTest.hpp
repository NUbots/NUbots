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

#include "EmissionCatcher.hpp"

namespace extension::moduletest {

    template <typename Module>
    class ModuleTest : public NUClear::PowerPlant {
    public:
        ModuleTest() = delete;

        // TODO: somehow disable the reactions which aren't on<Trigger<..>>, such as on<Every<..>> or on<Always>
        explicit ModuleTest(const bool start_powerplant_automatically = true)
            : NUClear::PowerPlant(get_single_thread_config()) {
            install<EmissionCatcher>();
            install<Module>();
            if (start_powerplant_automatically) {
                startup_manually();
            }
        }
        ~ModuleTest() {
            started = false;
            catcher.unbind_all();
            shutdown();
        }
        void startup_manually() {
            INFO("Starting up ModuleTest power plant.");
            started = true;
            start();
        }
        void shutdown_manually() {
            INFO("Shutting down ModuleTest power plant. Further use will require a restart.");
            started = false;
            catcher.unbind_all();
            shutdown();
        }

        template <typename MessageType>
        void bind_catcher_for_next_reaction(std::shared_ptr<MessageType> message) {
            auto binding_function =
                [message](NUClear::Reactor& emission_catcher) -> NUClear::threading::ReactionHandle {
                return emission_catcher.on<NUClear::dsl::word::Trigger<MessageType>>().then(  //
                    [message](const MessageType& emitted_message) {                           //
                        *message = emitted_message;
                    });
            };
            emit(std::make_unique<extension::moduletest::EmissionBind>(binding_function));
        }

        ModuleTest(ModuleTest& other)  = delete;
        ModuleTest(ModuleTest&& other) = delete;
        ModuleTest& operator=(ModuleTest& other) = delete;
        ModuleTest&& operator=(ModuleTest&& other) = delete;

    private:
        static const NUClear::PowerPlant::Configuration get_single_thread_config() {
            NUClear::PowerPlant::Configuration cfg;
            cfg.thread_count = 1;
            return cfg;
        }

        extension::moduletest::EmissionCatcher catcher;
        bool started = false;  // TODO: throw if we do something we shouldn't, based on this bool
    };

}  // namespace extension::moduletest

#endif  // EXTENSION_MODULETEST_HPP
