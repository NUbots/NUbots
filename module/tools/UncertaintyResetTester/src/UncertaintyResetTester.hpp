/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef MODULE_TOOLS_UNCERTAINTYRESETTESTER_HPP
#define MODULE_TOOLS_UNCERTAINTYRESETTESTER_HPP

#include <Eigen/Core>
#include <chrono>
#include <nuclear>
#include <vector>

#include "message/nbs/player/Player.hpp"

namespace module::tools {

    /**
     * @brief Times each `UncertaintyResetFieldLocalisation` → `FinishReset` round-trip emitted by
     *        `localisation::FieldLocalisationNLopt` during NBS playback. Optionally provokes resets
     *        by periodically pushing the filter to a deliberately bad pose via `PenaltyReset`.
     */
    class UncertaintyResetTester : public NUClear::Reactor {
    private:
        struct Config {
            /// @brief Playback mode used by the nbs::Player
            message::nbs::player::PlaybackMode playback_mode;
            /// @brief Messages to play back from the NBS file(s)
            std::vector<std::string> messages{};
            /// @brief Period in seconds to force the filter into a bad pose. 0 disables forcing.
            double force_reset_period = 0.0;
            /// @brief Pose (x, y, theta) that the filter is forced into when `force_reset_period > 0`
            Eigen::Vector3d force_reset_position = Eigen::Vector3d::Zero();
        } config;

        /// @brief Wall-clock time the most recent `UncertaintyResetFieldLocalisation` was observed
        std::chrono::steady_clock::time_point reset_started{};
        /// @brief True between an `UncertaintyResetFieldLocalisation` and its matching `FinishReset`
        bool reset_in_progress = false;
        /// @brief Durations (seconds) of every observed reset, used for end-of-run statistics
        std::vector<double> durations_s{};

    public:
        explicit UncertaintyResetTester(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_UNCERTAINTYRESETTESTER_HPP
