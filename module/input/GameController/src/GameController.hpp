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
 * Copyright 2013 NUbots <nubots@nubots.net>
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
        static constexpr const uint SUPPORTED_VERSION       = 12;
        static constexpr const uint PLAYERS_PER_TEAM        = 6;
        static constexpr const uint ACTIVE_PLAYERS_PER_TEAM = 4;
        static constexpr const uint NUM_TEAMS               = 2;

        uint receive_port;
        uint send_port;
        uint TEAM_ID;
        uint PLAYER_ID;
        uint BROADCAST_IP = 0xFFFFFFFF;
        std::string udp_filter_address;
        std::set<std::string> ignored_ip_addresses;

        bool selfPenalised = true;
        ReactionHandle listenHandle;

        gamecontroller::GameControllerPacket packet;
        gamecontroller::Mode mode;

        void resetState();
        void process(const message::input::GameState& oldGameState,
                     const gamecontroller::GameControllerPacket& oldPacket,
                     const gamecontroller::GameControllerPacket& newPacket);
        void sendReplyMessage(const gamecontroller::ReplyMessage& message);
        [[nodiscard]] const gamecontroller::Team& getOwnTeam(const gamecontroller::GameControllerPacket& state) const;
        [[nodiscard]] const gamecontroller::Team& getOpponentTeam(
            const gamecontroller::GameControllerPacket& state) const;
        [[nodiscard]] static message::input::GameState::Data::PenaltyReason getPenaltyReason(
            const gamecontroller::PenaltyState& penaltyState);

        [[nodiscard]] static std::string ipAddressIntToString(const uint32_t& ipAddr);

    public:
        explicit GameController(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::input

#endif  // MODULES_INPUT_GAMECONTROLLER_HPP
