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
#include <thread>

#include "EmissionCatcher.hpp"

namespace extension::moduletest {

    template <typename Module>
    class ModuleTest : public NUClear::PowerPlant {
    public:
        // TODO: somehow disable the reactions which aren't on<Trigger<..>>, such as on<Every<..>> or on<Always>
        ModuleTest() : NUClear::PowerPlant(get_single_thread_config()) {
            install<EmissionCatcher>();
            install<Module>();
        }
        ~ModuleTest() {
            shutdown();
        }
        void start_test() {

            INFO("Starting up ModuleTest power plant.");
            start();
        }

        template <typename MessageType>
        void bind_catcher_for_next_reaction(std::shared_ptr<MessageType> message) {
            auto binding_function = [message](EmissionCatcher& emission_catcher) -> NUClear::threading::ReactionHandle {
                return emission_catcher.on<NUClear::dsl::word::Trigger<MessageType>>().then(
                    [&emission_catcher, message](const MessageType& emitted_message) {  //
                        *message = emitted_message;
                        emission_catcher.num_reactions_left--;
                        if (emission_catcher.num_reactions_left <= 0) {
                            emission_catcher.powerplant.shutdown();
                        }
                    });
            };
            emit(std::make_unique<extension::moduletest::EmissionBind>(binding_function, num_reactions_this_test));
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

        ssize_t num_reactions_this_test = 0;
    };


}  // namespace extension::moduletest

#endif  // EXTENSION_MODULETEST_HPP
