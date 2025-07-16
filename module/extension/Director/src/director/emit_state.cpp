/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <nuclear>
#include <set>

#include "Director.hpp"

namespace module::extension {

    using message::behaviour::state::DirectorState;

    void Director::emit_state() {

        // Create a lambda that will update a value and set the updated flag
        auto update = [this](auto& current_value, const auto& new_value) {
            if (current_value != new_value) {
                current_value = new_value;
                state_changed = true;
            }
        };

        // Go through all the groups and update their information
        for (const auto& [group_id, group] : groups) {

            auto& proto_group = director_state.groups.at(group_id.hash_code());

            update(proto_group.done, group.done);
            update(proto_group.active_provider, group.active_provider ? group.active_provider->id : 0);

            std::vector<uint64_t> watcher_ids;
            watcher_ids.reserve(group.watchers.size());
            for (const auto& watcher : group.watchers) {
                watcher_ids.push_back(watcher->requester_id);
            }
            update(proto_group.watchers, watcher_ids);

            // Rebuild subtasks for this group
            std::vector<uint64_t> subtask_ids;
            subtask_ids.reserve(group.subtasks.size());
            for (const auto& subtask : group.subtasks) {
                subtask_ids.push_back(subtask->requester_task_id);
            }
            update(proto_group.subtasks, subtask_ids);
        }

        // We rebuild all the tasks each time from all subtasks
        std::map<uint64_t, DirectorState::DirectorTask> new_tasks;
        for (const auto& group_entry : groups) {
            for (const auto& subtask : group_entry.second.subtasks) {
                if (subtask) {
                    auto& proto_task                 = new_tasks[subtask->requester_task_id];
                    proto_task.name                  = subtask->name;
                    proto_task.target_group          = subtask->type.hash_code();
                    proto_task.requester_provider_id = subtask->requester_id;
                    proto_task.priority              = subtask->priority;
                    proto_task.optional              = subtask->optional;
                }
            }
        }

        update(director_state.tasks, new_tasks);

        // If we updated the state, emit it
        if (state_changed) {
            state_changed = false;
            emit(std::make_unique<DirectorState>(director_state));
        }
    }
}  // namespace module::extension
