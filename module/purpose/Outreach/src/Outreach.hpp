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

#ifndef MODULE_PURPOSE_OUTREACH_HPP
#define MODULE_PURPOSE_OUTREACH_HPP

#include <Eigen/Core>
#include <nuclear>

#include "extension/Behaviour.hpp"

namespace module::purpose {

    class Outreach : public ::extension::behaviour::BehaviourReactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Minimum YOLO confidence for a bounding box to count as a person
            double confidence_threshold = 0.0;
            /// @brief How long after the last person detection we keep tracking them
            NUClear::clock::duration person_timeout{};
            /// @brief Proportional gain from the person's bearing to the walk's rotational velocity
            double turn_gain = 0.0;
            /// @brief Maximum rotational velocity used to turn towards the person
            double max_turn_speed = 0.0;
            /// @brief Bearing error below which we stop turning, to stop the robot shuffling on the spot
            double turn_deadband = 0.0;
            /// @brief Minimum time between the start of one wave and the start of the next
            NUClear::clock::duration wave_cooldown{};
            /// @brief Duration modifier for the Wave script; smaller is faster
            double wave_speed = 1.0;
            /// @brief Priority of the Look/LookAround tasks
            int look_priority = 0;
            /// @brief Priority of the Walk task
            int walk_priority = 0;
            /// @brief Priority of the WaveAtPerson task
            int wave_priority = 0;
            /// @brief Priority of the Outreach task
            int outreach_priority = 0;
            /// @brief Priority of the FallRecovery task
            int fall_recovery_priority = 0;
        } cfg;

        /// @brief Unit vector from the camera to the middle of the last person seen, in torso space
        Eigen::Vector3d uPCt = Eigen::Vector3d::UnitX();

        /// @brief When the last person was seen
        NUClear::clock::time_point last_seen{};

        /// @brief When the last wave was started
        NUClear::clock::time_point last_wave{};

        /// @brief True if the last person detection is recent enough to still be worth tracking. This is a check on
        /// how stale uPCt is, not on whether anyone is in the current image
        bool person_seen_recently() const {
            return NUClear::clock::now() - last_seen < cfg.person_timeout;
        }

        /// @brief The rate the tasks will emit, to drive the rest of the system
        static constexpr size_t BEHAVIOUR_UPDATE_RATE = 10;

    public:
        /// @brief Called by the powerplant to build and setup the Outreach reactor.
        explicit Outreach(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_OUTREACH_HPP
