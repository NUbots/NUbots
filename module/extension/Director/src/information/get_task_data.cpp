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

    std::shared_ptr<void> Director::_get_task_data(const uint64_t& reaction_id) {
        // Still need this lock because of withs etc :(
        // Perhaps though what should happen is that an emit for the director state should be emitted before so these
        // can look it up in the cache
        std::lock_guard<std::recursive_mutex> lock(director_mutex);

        // How did we get here?
        if (!providers.contains(reaction_id)) {
            return nullptr;
        }

        // Get the provider and group for the reaction
        auto provider = providers.at(reaction_id);
        auto group    = provider->group;

        // Only the active provider is allowed to have data
        if (provider != group.active_provider) {
            return nullptr;
        }

        // If the task is of the wrong type we can't return it
        if (group.active_task->type != provider->type) {
            return nullptr;
        }

        // Return the data
        return group.active_task->data;
    }

}  // namespace module::extension
