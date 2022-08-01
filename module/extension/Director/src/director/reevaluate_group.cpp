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

#include "Director.hpp"

namespace module::extension {

    using provider::Provider;

    void Director::reevaluate_group(provider::ProviderGroup& group) {

        // We need to take a copy of the watchers before we start as reevaluting the group will alter this order
        std::vector<std::shared_ptr<const BehaviourTask>> watchers = group.watchers;

        // Sort the interested parties by priority with highest first
        std::sort(watchers.begin(), watchers.end(), [this](const auto& a, const auto& b) {
            return !challenge_priority(a, b);
        });

        // Store our initial task to see if it changed later
        auto initial_task = group.active_task;

        // Go through each of the watchers and try to re-run them and their siblings
        for (const auto& t : watchers) {

            // Check if we have the priority to override the currently active task
            if (t != group.active_task && challenge_priority(group.active_task, t)) {
                // Find the provider group that requested this task and reevaluate it
                // Maybe it's blocked on something else and that's why it's not our active task
                auto p  = providers.at(t->requester_id);
                auto& g = p->group;
                reevaluate_group(g);

                // If running this task pack claimed the active task we can stop looking
                if (group.active_task == t) {
                    break;
                }
            }
        }

        // If we ended up with the same task that we started with at the end of all this we might still need to change
        // state because of changes in when/needs that required this reevaluation in the first place
        // We assume that if we changed tasks because of this loop that whoever changed our task took care of running
        // the provider in this group
        // Also if we have traversed all the way up to a root provider we just run it again
        if (group.active_provider->classification == Provider::Classification::ROOT
            || (group.active_task != nullptr && group.active_task == initial_task)) {
            run_task_pack(TaskPack(group.active_provider, group.subtasks));
        }
    }

}  // namespace module::extension
