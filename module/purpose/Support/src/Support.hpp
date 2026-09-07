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
#ifndef MODULE_PURPOSE_SUPPORT_HPP
#define MODULE_PURPOSE_SUPPORT_HPP

#include <nuclear>
#include <Eigen/Core>
#include <map>
#include <optional>
#include <string>

#include "extension/Behaviour.hpp"

#include "message/input/GameState.hpp"
#include "message/localisation/Ball.hpp"
#include "message/localisation/Field.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/GlobalConfig.hpp"

namespace module::purpose {

    class Support : public ::extension::behaviour::BehaviourReactor {

    private:
        /// @brief Stores the offset, attraction, and minimum x for a robot's formation slot
        struct RobotSlot {
            /// @brief Base position on the field in field coordinates (x, y)
            Eigen::Vector2d offset{0.0, 0.0};
            /// @brief How much the position tracks the ball (x, y scale factors)
            Eigen::Vector2d attraction{0.0, 0.0};
            /// @brief Minimum x clamp
            double min_x{-10.0};
            /// @brief Maximum x clamp
            double max_x{10.0};
            /// @brief Minimum y clamp
            double min_y{-10.0};
            /// @brief Maximum y clamp
            double max_y{10.0};
        };

        /// @brief Stores configuration values
        struct Config {
            /// @brief Formation slots per game mode, keyed by mode name then player ID
            std::map<std::string, std::map<int, RobotSlot>> modes;
            /// @brief Translational/angular error threshold (metres/radians) within which Support
            ///        considers itself arrived at its target position. Passed to WalkToFieldPosition as
            ///        a per-task override (so only Support gets this radius) and reused locally to gate
            ///        when Support starts looking at the ball.
            double stop_threshold = 0.0;
            /// @brief Wider hysteresis threshold once stopped, passed as WalkToFieldPosition's
            ///        stopped-threshold override.
            double stopped_threshold = 0.0;
        } cfg;

        /// @brief Resolve this robot's formation slot for the given mode, falling back to the
        ///        normal_play formation when the mode is not defined. Returns nullptr when neither
        ///        the mode nor the fallback defines a slot for this robot (e.g. Formation.yaml has
        ///        not been loaded yet), so callers never dereference a past-the-end iterator.
        /// @param mode_name the formation mode to look up
        /// @param player_id the robot's player ID
        /// @return pointer to the matching slot, or nullptr if none applies
        const RobotSlot* find_slot(const std::string& mode_name, int player_id) const;

        /// @brief Compute what this robot's support position would be right now, given the current ball,
        ///        field, game state and formation config - regardless of whether this robot is actually
        ///        assigned the Support purpose. Used both by the active Support behaviour (to know where to
        ///        walk) and by an always-on reactor that lets NUsight show a live preview of this position
        ///        even when some other robot is currently the one supporting.
        /// @return the computed field-space position (x, y, 0), or std::nullopt if no formation slot applies
        ///         (e.g. Formation.yaml has not been loaded yet)
        std::optional<Eigen::Vector3d> calculate_support_position(
            const std::shared_ptr<const message::localisation::Ball>& ball,
            const message::localisation::Field& field,
            const message::input::GameState& game_state,
            const message::support::GlobalConfig& global_config,
            const message::support::FieldDescription& fd) const;

    public:
        /// @brief Called by the powerplant to build and setup the Support reactor.
        explicit Support(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::purpose

#endif  // MODULE_PURPOSE_SUPPORT_HPP
