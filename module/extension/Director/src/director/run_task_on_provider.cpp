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
 * Copyright 2022 NUbots <nubots@nubots.net>
 */

#include "Director.hpp"

namespace module::extension {

    using component::DirectorTask;
    using component::Provider;
    using ::extension::behaviour::RunInfo;

    void Director::run_task_on_provider(const std::shared_ptr<DirectorTask>& task,
                                        const std::shared_ptr<component::Provider>& provider,
                                        const RunInfo::RunReason& run_reason) {

        // Update the active provider and task
        auto& group              = provider->group;
        bool run_start_providers = group.active_provider == nullptr;
        group.active_task        = task;

        // If there was no active provider, then this is the first time running this group
        // Therefore we should run the "start" providers
        if (run_start_providers) {
            for (auto& provider : group.providers) {
                if (provider->classification == Provider::Classification::START) {
                    // We have to swap to this as the active provider so it can actually run
                    group.active_provider = provider;

                    auto lock = hold_run_reason(RunInfo::RunReason::STARTED);
                    auto task = provider->reaction->get_task();
                    if (task) {
                        powerplant.submit(std::move(task), true);
                    }
                }
            }
        }

        // Set the active provider ready for running
        group.active_provider = provider;

        // Run the reaction
        auto lock          = hold_run_reason(run_reason);
        auto reaction_task = provider->reaction->get_task();
        if (reaction_task) {
            powerplant.submit(std::move(reaction_task));
        }
    }

}  // namespace module::extension
