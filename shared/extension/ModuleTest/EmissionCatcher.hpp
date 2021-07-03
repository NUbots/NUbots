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

#ifndef EXTENSION_MODULETEST_EMISSIONCATCHER_HPP
#define EXTENSION_MODULETEST_EMISSIONCATCHER_HPP

#include <catch.hpp>
#include <nuclear>

namespace extension::moduletest {

    template <typename MessageType>
    struct EmissionBind {
        EmissionBind() = delete;
        explicit EmissionBind(std::shared_ptr<MessageType> message_) : message(message_){};
        std::shared_ptr<MessageType> message;
    };

    class EmissionCatcher : public NUClear::Reactor {
    public:
        template <typename MessageType>
        explicit EmissionCatcher(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
            on<Trigger<extension::moduletest::EmissionBind<MessageType>>>().then(
                // TODO: This doesn't work, because this wants me to know MessageType when I construct the
                // EmissionCatcher, but what I want to do is pass in different types after this reactor has been
                // constructed. So MessageType will change after each expected emission is loaded in. This set up does
                // not allow that though
                [this](const EmissionBind<MessageType>& emission_bind) {  //
                    bind_catcher(emission_bind.message);
                });
        }

        template <typename MessageType>
        void bind_catcher(std::shared_ptr<MessageType> message) {
            INFO("Binding message pointer to catch emitted message.");
            auto handle = on<MessageType>().then([message](const MessageType& emitted_message) {  //
                *message = emitted_message;
            });
            handles.push_back(handle);
        }

        inline void unbind_all() {
            for (auto& handle : handles) {
                handle.unbind();
            }
            handles.clear();
        }

    private:
        std::vector<NUClear::threading::ReactionHandle> handles{};
    };
}  // namespace extension::moduletest

#endif  // EXTENSION_MODULETEST_EMISSIONCATCHER
