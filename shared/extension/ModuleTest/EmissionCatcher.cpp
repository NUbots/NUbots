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

#include "EmissionCatcher.hpp"

#include <catch.hpp>

namespace extension::moduletest {
    EmissionCatcher::EmissionCatcher(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {}

    template <typename MessageType>
    void EmissionCatcher::bind_catcher(std::shared_ptr<MessageType> message) {
        INFO("Binding message pointer to catch emitted message.");
        auto handle = on<MessageType>().then([message](const MessageType& emitted_message) {  //
            *message = emitted_message;
        });
        handles.push_back(handle);
    }

    void EmissionCatcher::unbind_all() {
        for (auto& handle : handles) {
            handle.unbind();
        }
        handles.clear();
    }
}  // namespace extension::moduletest
