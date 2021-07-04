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
#include <functional>
#include <nuclear>

namespace extension::moduletest {
    using NUClear::threading::ReactionHandle;

    struct EmissionBind {
        EmissionBind() = delete;
        explicit EmissionBind(const std::function<ReactionHandle(NUClear::Reactor& emission_catcher)>& b_fn)
            : binding_function(b_fn) {}
        std::function<ReactionHandle(NUClear::Reactor& emission_catcher)> binding_function;
    };

    struct UnbindAllCommand {};

    class EmissionCatcher : public NUClear::Reactor {
    public:
        inline explicit EmissionCatcher(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {
            on<Trigger<EmissionBind>>().then([this](const EmissionBind& emission_bind) {
                auto binding_function = emission_bind.binding_function;
                auto handle           = binding_function(*this);
                handles.push_back(handle);
            });
            on<Trigger<UnbindAllCommand>>().then([this](const UnbindAllCommand& /*cmd*/) { unbind_all(); });
        };

        inline void unbind_all() {
            for (auto& handle : handles) {
                handle.unbind();
            }
            handles.clear();
        }

    private:
        std::vector<ReactionHandle> handles{};
    };
}  // namespace extension::moduletest

#endif  // EXTENSION_MODULETEST_EMISSIONCATCHER
