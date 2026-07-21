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

#ifndef UTILITY_SLAM_EVENT_HPP
#define UTILITY_SLAM_EVENT_HPP


#include "system/SystemBase.hpp"

namespace utility::slam {

    // Bring types into scope
    using system::SystemBase;

    /**
     * @brief Base class for all events in the system.
     *
     * This class represents an abstract event that can be processed by the system.
     * Concrete event types should inherit from this class and implement the update method.
     */
    class Event {
    public:
        /**
         * @brief Construct a new Event object.
         * @param time The time at which the event occurs.
         */
        Event(double time);

        /**
         * @brief Destroy the Event object.
         */
        virtual ~Event();

        /**
         * @brief Process the event in the given system.
         * @param system The system in which to process the event.
         */
        void process(SystemBase& system);

    protected:
        /**
         * @brief Update the system based on this event.
         * @param system The system to update.
         *
         * This pure virtual function must be implemented by derived classes
         * to define the specific behavior of the event.
         */
        virtual void update(SystemBase& system) = 0;

        double time_;  ///< The time at which the event occurs.
    };

}  // namespace utility::slam

#endif  // UTILITY_SLAM_EVENT_HPP
