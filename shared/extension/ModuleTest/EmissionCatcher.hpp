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

    class EmissionCatcher;

    struct EmissionBind {
        EmissionBind() = delete;
        explicit EmissionBind(const std::function<ReactionHandle(EmissionCatcher& emission_catcher)>& b_fn,
                              const ssize_t& num_reactions_bound_)
            : binding_function(b_fn), num_reactions_bound(num_reactions_bound_) {}
        std::function<ReactionHandle(EmissionCatcher& emission_catcher)> binding_function;
        ssize_t num_reactions_bound = 0;
    };

    class EmissionCatcher : public NUClear::Reactor {
    public:
        inline explicit EmissionCatcher(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {

            on<Trigger<std::function<void()>>>().then([this](const std::function<void()>& emit_function) {  //
                emitter = emit_function;
            });

            on<Trigger<EmissionBind>>().then([this](const EmissionBind& emission_bind) {
                auto binding_function = emission_bind.binding_function;
                auto handle           = binding_function(*this);
                handles.push_back(handle);
                num_reactions_left = emission_bind.num_reactions_bound;
                FAIL("emission bound");
            });

            on<Startup>().then([this]() {
                FAIL("on startup triggered in catcher");
                emitter();
            });

            on<Shutdown>().then([this]() { unbind_all(); });
        };

        inline void unbind_all() {
            for (auto& handle : handles) {
                handle.unbind();
            }
            handles.clear();
        }
        /// Serves as a counter of emissions yet to be captured. After this hits zero again, shutdown is triggered
        ssize_t num_reactions_left = 0;

    private:
        std::vector<ReactionHandle> handles{};
        std::function<void()> emitter{};
    };
}  // namespace extension::moduletest

#endif  // EXTENSION_MODULETEST_EMISSIONCATCHER_HPP
