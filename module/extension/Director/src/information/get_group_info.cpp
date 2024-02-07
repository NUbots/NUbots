/*
 * MIT License
 *
 * Copyright (c) 2022 NUbots
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

#include "Director.hpp"

namespace module::extension {

    using ::extension::behaviour::GroupInfo;

    GroupInfo Director::_get_group_info(const uint64_t& reaction_id,
                                        const std::type_index& type,
                                        const std::type_index& root_type) {
        std::lock_guard<std::recursive_mutex> lock(director_mutex);

        // If the reaction is a non-Provider, then we need to get the root Provider
        const auto& provider =
            providers.contains(reaction_id) ? providers.at(reaction_id) : get_root_provider(root_type);

        if (groups.contains(type)) {
            auto& group = groups.at(type);

            // Check if the task is active
            if (group.active_task != nullptr) {
                // Check if the task is being run by this reaction
                if (group.active_task->requester_id == provider->id) {
                    return GroupInfo{GroupInfo::RunState::RUNNING, group.done};
                }
            }

            // Check if this task is in the reaction's task list
            // If it is, then it is waiting to be run
            for (const auto& task : provider->group.subtasks) {
                if (task->type == type) {
                    return GroupInfo{GroupInfo::RunState::QUEUED, group.done};
                }
            }

            // Not running or queued, so it must not have been requested yet
            return GroupInfo{GroupInfo::RunState::NO_TASK, groups.at(type).done};
        }
        else {
            throw std::runtime_error("No group with the requested type");
        }
    }

}  // namespace module::extension
