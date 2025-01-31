/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#ifndef EXTENSION_BEHAVIOUR_TASK_TYPES_HPP
#define EXTENSION_BEHAVIOUR_TASK_TYPES_HPP

#include <nuclear>

namespace extension::behaviour {

    /**
     * This is a special task that should be emitted when a Provider finishes the task it was given.
     * When this is emitted the director will re-execute the Provider which caused this task to run.
     *
     * ```
     * emit<Task>(std::make_unique<Done>());
     * ```
     */
    struct Done {};

    /**
     * This is a special task that should be emitted when a Provider doesn't want to change what it is doing.
     * When this is emitted the director will just continue with whatever was previously emitted by this provider.
     *
     * ```
     * emit<Task>(std::make_unique<Continue>());
     * ```
     */
    struct Continue {};

    /**
     * This is a special task that can be emitted to trigger the Provider to run again at a given time.
     *
     * ```
     * emit<Task>(std::make_unique<Wait>(NUClear::clock::now() + std::chrono::milliseconds(100)));
     * ```
     */
    struct Wait {
        /// The time at which the Provider should run again
        NUClear::clock::time_point time;

        /**
         * Create a new Wait task with the time at which the Provider should run again.
         *
         * @param time the time at which the Provider should run again
         */
        explicit Wait(const NUClear::clock::time_point& time) : time(time) {}
    };

}  // namespace extension::behaviour


#endif  // EXTENSION_BEHAVIOUR_TASK_TYPES_HPP
