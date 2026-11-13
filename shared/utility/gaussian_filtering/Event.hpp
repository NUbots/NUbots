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

#ifndef UTILITY_GAUSSIAN_FILTERING_EVENT_HPP
#define UTILITY_GAUSSIAN_FILTERING_EVENT_HPP


#include "system/SystemBase.hpp"

namespace utility::gaussian_filtering {

    using system::SystemBase;

    /// @brief Base class for events, which a system is predicted to and then updated by
    class Event {
    public:
        /// @param time The time at which the event occurs
        Event(double time);

        virtual ~Event();

        /// @brief Predict the system to the event time, then update it
        void process(SystemBase& system);

    protected:
        /// @brief Update the system based on this event
        virtual void update(SystemBase& system) = 0;

        double time_;  ///< The time at which the event occurs
    };

}  // namespace utility::gaussian_filtering

#endif  // UTILITY_GAUSSIAN_FILTERING_EVENT_HPP
