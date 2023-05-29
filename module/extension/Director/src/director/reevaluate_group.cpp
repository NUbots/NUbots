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

    bool Director::reevaluate_group(component::ProviderGroup& group) {

        // If we have traversed all the way up to a root provider we just run it again
        if (group.active_provider != nullptr
            && group.active_provider->classification == Provider::Classification::ROOT) {
            run_task_pack(TaskPack(group.active_provider, group.subtasks));
            return false;
        }

        // We need to take a copy of the watchers before we start as reevaluting the group will alter this order
        std::vector<std::shared_ptr<DirectorTask>> watchers = group.watchers;

        // Sort the interested parties by priority with highest first
        std::stable_sort(watchers.begin(), watchers.end(), [this](const auto& a, const auto& b) {
            // If b wins against a, then we swap
            return challenge_priority(b, a);
        });

        // Store our initial task to see if it changed later
        auto initial_task = group.active_task;

        // Go through each of the watchers and try to re-run them and their siblings
        for (const auto& t : watchers) {

            // Check if we have the priority to override the currently active task
            if (challenge_priority(group.active_task, t)) {
                // Find the provider group that requested this task and reevaluate it
                // Maybe it's blocked on something else and that's why it's not our active task
                reevaluate_group(providers.at(t->requester_id)->group);

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
        if (group.active_task != nullptr && group.active_task == initial_task) {
            run_task_pack(TaskPack(group.active_provider, group.subtasks));
        }

        // If we aren't running any task we need to clear our watch handles
        if (group.active_task == nullptr) {
            group.watch_handles.clear();
        }

        return group.active_task != initial_task;
    }

    void Director::reevaluate_children(component::ProviderGroup& group) {
        // Reevaluate this group
        bool changed = reevaluate_group(group);

        // If the task we are working on did not change we need to look deeper
        if (!changed) {
            for (const auto& t : group.subtasks) {
                auto& g = groups.at(t->type);

                // Only bother if we are the active task
                if (g.active_task == t) {
                    reevaluate_children(g);
                }
            }
        }
    }

}  // namespace module::extension
