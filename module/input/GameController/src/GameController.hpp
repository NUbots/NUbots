/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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

#ifndef MODULES_INPUT_GAMECONTROLLER_HPP
#define MODULES_INPUT_GAMECONTROLLER_HPP

#include <nuclear>

#include "GameControllerData.hpp"

#include "message/input/GameEvents.hpp"
#include "message/input/GameState.hpp"

namespace module::input {

    /**
     * Monitors the match Game Controller
     *
     * @author Brendan Annable
     * @author Jordan Johnson
     * @author Trent Houliston
     */
    class GameController : public NUClear::Reactor {
    private:
        static constexpr const uint SUPPORTED_VERSION       = 19;
        static constexpr const uint PLAYERS_PER_TEAM        = 6;
        static constexpr const uint ACTIVE_PLAYERS_PER_TEAM = 4;
        static constexpr const uint NUM_TEAMS               = 2;

        /// @brief Port to receive GameController packets on
        uint receive_port{0};

        /// @brief Port to send reply packets to the GameController on
        uint send_port{0};

        /// @brief Team ID of the robot's team
        uint TEAM_ID{0};

        /// @brief Player ID of this robot
        uint PLAYER_ID{0};

        /// @brief The address to use to send reply packets back to game controller (unicast)
        std::string game_controller_address{};

        /// @brief Optional IP address to filter incoming packets by source, ignoring all others if set
        std::string udp_filter_address{};

        /// @brief Set of IP addresses that have been ignored due to not matching the udp_filter_address
        std::set<std::string> ignored_ip_addresses{};

        /// @brief Indicates if the current player is penalised
        bool self_penalised{true};

        /// @brief Handle for our registered UDP receive callback
        ReactionHandle listen_handle;

        /// @brief Latest packet received from the GameController
        gamecontroller::GameControllerPacket packet{};

        /// @brief Current game phase from the GameController packet
        gamecontroller::GamePhase game_phase{};

        /// @brief Current set play from the GameController packet
        gamecontroller::SetPlay set_play{};

        /// @brief Resets the game state to initial values
        void reset_state();

        /// @brief Processes a new GameController packet and emits state changes
        void process(const message::input::GameState& old_game_state,
                     const gamecontroller::GameControllerPacket& old_packet,
                     const gamecontroller::GameControllerPacket& new_packet);

        /// @brief Sends a reply packet to the GameController
        void send_reply_message();

        /// @brief Returns the team from the packet that matches our team ID
        [[nodiscard]] const gamecontroller::Team& get_own_team(const gamecontroller::GameControllerPacket& state) const;

        /// @brief Returns the team from the packet that does not match our team ID
        [[nodiscard]] const gamecontroller::Team& get_opponent_team(
            const gamecontroller::GameControllerPacket& state) const;

        /// @brief Converts a gamecontroller::TeamColour to a GameState::TeamColour::Value
        [[nodiscard]] static message::input::GameState::TeamColour::Value get_team_colour(
            const gamecontroller::TeamColour& colour);

        /// @brief Converts a gamecontroller::PenaltyState to a GameState::PenaltyReason
        [[nodiscard]] static message::input::GameState::PenaltyReason get_penalty_reason(
            const gamecontroller::PenaltyState& penalty_state);

        /// @brief Converts an integer IP address to a string
        [[nodiscard]] static std::string ip_address_int_to_string(const uint32_t& ip_addr);

    public:
        /// @brief Called by the powerplant to build and setup the GameController reactor
        explicit GameController(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::input

#endif  // MODULES_INPUT_GAMECONTROLLER_HPP
