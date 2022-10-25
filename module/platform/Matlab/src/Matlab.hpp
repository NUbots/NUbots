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

#ifndef MODULE_PLATFORM_MATLAB_HPP
#define MODULE_PLATFORM_MATLAB_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <map>
#include <mutex>
#include <nuclear>
#include <string>
#include <vector>

#include "utility/input/ServoID.hpp"

namespace module::platform {

    using utility::input::ServoID;

    class Matlab : public NUClear::Reactor {
    private:
        /// @brief The id registered in the subsumption system for this module
        const size_t subsumption_id;

        /// @brief The update frequency for requesting data from Matlab
        static constexpr int UPDATE_FREQUENCY = 1;

        /// @brief
        int server_fd;

        /// @brief
        struct sockaddr_in address;

        /// @brief Number of servos in the message from Matlab
        static const int n_servos = 18;

        /// @brief The order of servo ids in the message from Matlab
        const std::array<ServoID, n_servos> servo_ids = {ServoID::R_ANKLE_ROLL,
                                                         ServoID::R_ANKLE_PITCH,
                                                         ServoID::R_KNEE,
                                                         ServoID::R_HIP_PITCH,
                                                         ServoID::R_HIP_ROLL,
                                                         ServoID::R_HIP_YAW,
                                                         ServoID::L_HIP_YAW,
                                                         ServoID::L_HIP_ROLL,
                                                         ServoID::L_HIP_PITCH,
                                                         ServoID::L_KNEE,
                                                         ServoID::L_ANKLE_PITCH,
                                                         ServoID::L_ANKLE_ROLL,
                                                         ServoID::R_SHOULDER_PITCH,
                                                         ServoID::R_SHOULDER_ROLL,
                                                         ServoID::R_ELBOW,
                                                         ServoID::L_ELBOW,
                                                         ServoID::L_SHOULDER_ROLL,
                                                         ServoID::L_SHOULDER_PITCH};

        struct Config {
            Config() = default;

            /// @brief Port of the server
            uint16_t tcp_port;

            /// @brief The gains for servo commands sent from Matlab
            double servo_gain;

            /// @brief The priority of the Matlab module
            float matlab_priority;
        } cfg;

        /// @brief Updates the priority of the module by emitting an ActionPriorities message
        /// @param priority The priority used in the ActionPriorities message
        void update_priority(const float& priority);

    public:
        /// @brief Called by the powerplant to build and setup the webots reactor
        explicit Matlab(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::platform

#endif  // MODULE_PLATFORM_MATLAB_HPP
