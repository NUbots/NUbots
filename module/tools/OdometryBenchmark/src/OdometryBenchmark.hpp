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
#ifndef MODULE_TOOLS_ODOMETRYBENCHMARK_HPP
#define MODULE_TOOLS_ODOMETRYBENCHMARK_HPP

#include <nuclear>

#include "message/nbs/player/Player.hpp"

#include "utility/support/ProgressBar.hpp"

namespace module::tools {

    class OdometryBenchmark : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Playback mode
            message::nbs::player::PlaybackMode mode;
            /// @brief Messages to play
            std::vector<std::string> messages{};
        } config;

        /// @brief The initialised ground truth Hfw
        Eigen::Isometry3d ground_truth_Hfw;

        /// @brief Whether the ground truth Hfw has been initialised
        bool ground_truth_initialised = false;

        /// @brief The total error in the odometry translation
        double total_odometry_translation_error = 0.0;

        /// @brief The total error in the odometry rotation
        double total_odometry_rotation_error = 0.0;

        /**
         * @defgroup ErrorAccumulations
         * @brief Error accumulations for each individual DoF
         */
        ///@{
        double total_error_x     = 0.0;
        double total_error_y     = 0.0;
        double total_error_z     = 0.0;
        double total_error_roll  = 0.0;
        double total_error_pitch = 0.0;
        double total_error_yaw   = 0.0;
        ///@}

        /// @brief Count of the number of messages
        size_t count = 0;

    public:
        /// @brief Called by the powerplant to build and setup the OdometryBenchmark reactor.
        explicit OdometryBenchmark(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_ODOMETRYBENCHMARK_HPP
