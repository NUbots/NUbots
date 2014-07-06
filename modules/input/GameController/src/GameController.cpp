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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include <atomic>
#include "GameController.h"
#include "messages/input/gameevents/GameEvents.h"
#include "messages/support/Configuration.h"

extern "C" {
    #include <sys/socket.h>
    #include <arpa/inet.h>
}

namespace modules {
namespace input {

    using messages::support::Configuration;
    using namespace messages::input::gameevents;

    GameController::GameController(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), socket(0) {

        powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&GameController::run), this), std::bind(std::mem_fn(&GameController::kill), this)));

        on<Trigger<Configuration<GameController>>>([this](const Configuration<GameController>& config) {

            // TODO use an eventfd to allow changing the port dynamically

            TEAM_ID = config["teamId"].as<uint>();
            PLAYER_ID = config["playerId"].as<uint>();

            port = config["port"].as<uint>();
            sockaddr_in socketAddress;
            memset(&socketAddress, 0, sizeof(socketAddress));
            int broadcast = 1;
            socketAddress.sin_family = AF_INET;
            socketAddress.sin_port = htons(port);
            socketAddress.sin_addr.s_addr = INADDR_ANY;

            int newSocket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            setsockopt(newSocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
            ::bind(newSocket, reinterpret_cast<sockaddr*>(&socketAddress), sizeof(sockaddr));

            int oldSocket = socket.exchange(newSocket);

            if (oldSocket) {
                ::close(oldSocket);
            }
        });

    }

    void GameController::run() {
        sockaddr_in broadcastSocket;
        GameControllerPacket newState;
        unsigned slen = sizeof(sockaddr);

        while (listening) {
            recvfrom(socket, reinterpret_cast<char*>(&newState), sizeof(GameControllerPacket), 0, reinterpret_cast<sockaddr*>(&broadcastSocket), &slen);

            if (newState.version == SUPPORTED_VERSION) {
                process(state, newState);

                state = newState;

            }
        }
    }

    void GameController::process(GameControllerPacket oldState, GameControllerPacket newState) {
        // game score
        Team& oldOwnTeam = getOwnTeam(oldState);
        Team& newOwnTeam = getOwnTeam(newState);

        Team& oldOpponentTeam = getOpponentTeam(oldState);
        Team& newOpponentTeam = getOpponentTeam(newState);

        if (oldOwnTeam.score != newOwnTeam.score || oldOpponentTeam.score != newOpponentTeam.score) {
            // score update
            emit(std::make_unique<Score>(Score{newOwnTeam.score, newOpponentTeam.score}));

            if (oldOwnTeam.score > newOwnTeam.score) {
                // we scored! :D
                emit(std::make_unique<GoalScoredFor>(GoalScoredFor{newOwnTeam.score}));
            }

            if (oldOpponentTeam.score > newOpponentTeam.score) {
                // they scored :( boo
                emit(std::make_unique<GoalScoredAgainst>(GoalScoredAgainst{newOpponentTeam.score}));
            }
        }

        // Note: assumes playersPerTeam never changes
        for (uint i = 0; i < newState.playersPerTeam; i++) {
            auto& oldOwnPlayer = oldOwnTeam.players[i];
            auto& newOwnPlayer = newOwnTeam.players[i];

            auto& oldOpponentPlayer = oldOpponentTeam.players[i];
            auto& newOpponentPlayer = newOpponentTeam.players[i];

            // check if player on own team is penalised
            if (newOwnPlayer.penalised && !oldOwnPlayer.penalised) {
                auto unpenalisedTime = NUClear::clock::now() + std::chrono::seconds(newOwnPlayer.penalisedTimeLeft);
                if (i == PLAYER_ID) {
                    // self penalised :@
                    emit(std::make_unique<SelfPenalised>(SelfPenalised{i, unpenalisedTime}));
                } else {
                    // team mate penalised :'(
                    emit(std::make_unique<TeamMatePenalised>(TeamMatePenalised{i, unpenalisedTime}));
                }
            } else if (!newOwnPlayer.penalised && oldOwnPlayer.penalised) {
                if (i == PLAYER_ID) {
                    // self unpenalised :)
                    emit(std::make_unique<SelfUnpenalised>(SelfUnpenalised{i}));
                } else {
                    // team mate unpenalised :)
                    emit(std::make_unique<TeamMateUnpenalised>(TeamMateUnpenalised{i}));
                }
            }

            if (newOpponentPlayer.penalised && !oldOpponentPlayer.penalised) {
                auto unpenalisedTime = NUClear::clock::now() + std::chrono::seconds(newOpponentPlayer.penalisedTimeLeft);
                // opponent penalised :D
                emit(std::make_unique<OpponentPenalised>(OpponentPenalised{i, unpenalisedTime}));
            } else if (!newOpponentPlayer.penalised && oldOpponentPlayer.penalised) {
                // opponent unpenalised D:
                emit(std::make_unique<OpponentUnpenalised>(OpponentUnpenalised{i}));
            }
        }

        if (std::strcmp(oldOwnTeam.coachMessage, newOwnTeam.coachMessage) != 0) {
            // listen to the coach? o_O
            emit(std::make_unique<TeamCoachMessage>(TeamCoachMessage{std::string(newOwnTeam.coachMessage)}));
        }

        if (std::strcmp(oldOpponentTeam.coachMessage, newOpponentTeam.coachMessage) != 0) {
            // listen in on the enemy! >:D
            emit(std::make_unique<OpponentCoachMessage>(OpponentCoachMessage{std::string(newOpponentTeam.coachMessage)}));
        }

        /*
         * TODO:
         *
         * State changes
         * Subsate changes
         * Half changes
         * OurTeamColour
         * SecondsRemaining (combine with state changes)
         * SecondaryTimeRemaining (combine with state changes)
         * KickoffTeam
         * BallKickedOut
         */


    }

    Team& GameController::getOwnTeam(GameControllerPacket& state) {
        for (auto& team : state.teams) {
            if (team.teamId == TEAM_ID) {
                return team;
            }
        }

        throw std::runtime_error("Own team not found");
    }

    Team& GameController::getOpponentTeam(GameControllerPacket& state) {
        for (auto& team : state.teams) {
            if (team.teamId != TEAM_ID) {
                return team;
            }
        }

        throw std::runtime_error("No opponent teams not found"); // should never happen!
    }

    void GameController::kill() {
        listening = false;
        ::close(socket);
    }

}  // input
}  // modules
