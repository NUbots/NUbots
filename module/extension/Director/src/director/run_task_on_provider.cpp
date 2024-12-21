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

    using component::DirectorTask;
    using component::Provider;
    using ::extension::behaviour::GroupInfo;
    using ::extension::behaviour::RunReason;

    void Director::run_task_on_provider(const std::shared_ptr<DirectorTask>& task,
                                        const std::shared_ptr<component::Provider>& provider,
                                        const RunReason& run_reason) {

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

                    auto lock = group.update_data(groups, RunReason::STARTED);
                    powerplant.submit(provider->reaction->get_task());
                }
            }
        }

        // Set the active provider ready for running
        group.active_provider = provider;

        // Run the provider
        auto lock = group.update_data(groups, run_reason);
        powerplant.submit(provider->reaction->get_task());
    }

}  // namespace module::extension
