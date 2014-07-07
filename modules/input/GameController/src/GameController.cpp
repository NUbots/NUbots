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
    using TeamColourEvent = messages::input::gameevents::TeamColour;

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


        /*******************************************************************************************
         * Process score updates
         ******************************************************************************************/
        if (oldOwnTeam.score != newOwnTeam.score || oldOpponentTeam.score != newOpponentTeam.score) {
            // score update
            emit(std::make_unique<Score>(Score{newOwnTeam.score, newOpponentTeam.score}));

            if (oldOwnTeam.score > newOwnTeam.score) {
                // we scored! :D
                emit(std::make_unique<GoalScored<TEAM>>(GoalScored<TEAM>{newOwnTeam.score}));
            }

            if (oldOpponentTeam.score > newOpponentTeam.score) {
                // they scored :( boo
                emit(std::make_unique<GoalScored<OPPONENT>>(GoalScored<OPPONENT>{newOpponentTeam.score}));
            }
        }


        /*******************************************************************************************
         * Process penality updates
         ******************************************************************************************/
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
                    emit(std::make_unique<Penalisation<SELF>>(Penalisation<SELF>{i, unpenalisedTime}));
                } else {
                    // team mate penalised :'(
                    emit(std::make_unique<Penalisation<TEAM>>(Penalisation<TEAM>{i, unpenalisedTime}));
                }
            } else if (!newOwnPlayer.penalised && oldOwnPlayer.penalised) {
                if (i == PLAYER_ID) {
                    // self unpenalised :)
                    emit(std::make_unique<Unpenalisation<SELF>>(Unpenalisation<SELF>{i}));
                } else {
                    // team mate unpenalised :)
                    emit(std::make_unique<Unpenalisation<TEAM>>(Unpenalisation<TEAM>{i}));
                }
            }

            if (newOpponentPlayer.penalised && !oldOpponentPlayer.penalised) {
                auto unpenalisedTime = NUClear::clock::now() + std::chrono::seconds(newOpponentPlayer.penalisedTimeLeft);
                // opponent penalised :D
                emit(std::make_unique<Penalisation<OPPONENT>>(Penalisation<OPPONENT>{i, unpenalisedTime}));
            } else if (!newOpponentPlayer.penalised && oldOpponentPlayer.penalised) {
                // opponent unpenalised D:
                emit(std::make_unique<Unpenalisation<OPPONENT>>(Unpenalisation<OPPONENT>{i}));
            }
        }


        /*******************************************************************************************
         * Process coach messages
         ******************************************************************************************/
        if (std::strcmp(oldOwnTeam.coachMessage, newOwnTeam.coachMessage) != 0) {
            // listen to the coach? o_O
            emit(std::make_unique<CoachMessage<TEAM>>(CoachMessage<TEAM>{std::string(newOwnTeam.coachMessage)}));
        }

        if (std::strcmp(oldOpponentTeam.coachMessage, newOpponentTeam.coachMessage) != 0) {
            // listen in on the enemy! >:D
            emit(std::make_unique<CoachMessage<OPPONENT>>(CoachMessage<OPPONENT>{std::string(newOpponentTeam.coachMessage)}));
        }


        /*******************************************************************************************
         * Process half changes
         ******************************************************************************************/
        if (oldState.firstHalf != newState.firstHalf) {
            // half time
            emit(std::make_unique<HalfTime>(HalfTime{newState.firstHalf}));
        }

        /*******************************************************************************************
         * Process ball kicked out
         ******************************************************************************************/
        if (oldState.dropInTime >= 0 && oldState.dropInTime > newState.dropInTime)  {
            // ball was kicked out by dropInTeam
            auto time = NUClear::clock::now() - std::chrono::seconds(newState.dropInTime);
            if (newState.dropInTeam == newOwnTeam.teamColour) {
                // we kicked the ball out :S
                emit(std::make_unique<BallKickedOut<TEAM>>(BallKickedOut<TEAM>{time}));
            } else {
                // they kicked the ball out! ^_^
                emit(std::make_unique<BallKickedOut<OPPONENT>>(BallKickedOut<OPPONENT>{time}));
            }
        }


        /*******************************************************************************************
         * Process kick off team
         ******************************************************************************************/
        if (oldState.kickOffTeam != newState.kickOffTeam) {
            // new kick off team? :/
            Context team = newState.kickOffTeam == newOwnTeam.teamColour ? TEAM : OPPONENT;
            emit(std::make_unique<KickOffTeam>(KickOffTeam{team}));
        }


        /*******************************************************************************************
         * Process our team colour
         ******************************************************************************************/
        if (oldOwnTeam.teamColour != newOwnTeam.teamColour) {
            Colour colour = newOwnTeam.teamColour == TeamColour::CYAN ? Colour::CYAN : Colour::MAGENTA;
            emit(std::make_unique<TeamColourEvent>(TeamColourEvent{colour}));
        }


        /*******************************************************************************************
         * Process state/mode changes
         ******************************************************************************************/
        if(oldState.mode != newState.mode && newState.mode != Mode::TIMEOUT) {
            // Changed modes but not to timeout
            auto mode = newState.mode == Mode::PENALTY_SHOOTOUT ? GameMode::Mode::PENALTY_SHOOTOUT
                      : newState.mode == Mode::OVERTIME         ? GameMode::Mode::OVERTIME
                                                                : GameMode::Mode::NORMAL;
            emit(std::make_unique<GameMode>(GameMode{mode}));

        }

        if (oldState.mode != Mode::TIMEOUT && newState.mode == Mode::TIMEOUT) {
            // Change the game state to timeout
            auto time = NUClear::clock::now() + std::chrono::seconds(newState.secondaryTime);
            emit(std::make_unique<GameState<GamePhase::TIMEOUT>>(GameState<GamePhase::TIMEOUT>{time}));
        }
        else if (oldState.state != newState.state) {

            // State has changed, process it

            // TODO: handle timeouts and dropped balls
            switch (newState.state) {
                case State::INITIAL: {
                    emit(std::make_unique<GameState<GamePhase::INITIAL>>());
                    break;
                }
                case State::READY: {
                    /* Note: It was concluded that a specific 'dropped ball' event
                     * was not needed, but in the case that it is, this is how and
                     * where you would emit it:
                     */
                    // if (oldState.state == State::PLAYING) {
                    //     // DROPPED BALL
                    // }
                    auto time = NUClear::clock::now() + std::chrono::seconds(newState.secondaryTime);
                    emit(std::make_unique<GameState<GamePhase::READY>>(GameState<GamePhase::READY>{time}));
                    break;
                }
                case State::SET: {
                    emit(std::make_unique<GameState<GamePhase::SET>>());
                    break;
                }
                case State::PLAYING: {
                    auto endHalf = NUClear::clock::now() + std::chrono::seconds(newState.secsRemaining);
                    auto ballFree = NUClear::clock::now() + std::chrono::seconds(newState.secondaryTime);
                    emit(std::make_unique<GameState<GamePhase::PLAYING>>(GameState<GamePhase::PLAYING>{endHalf, ballFree}));
                    break;
                }
                case State::FINISHED: {
                    auto nextHalf = NUClear::clock::now() + std::chrono::seconds(newState.secsRemaining);
                    emit(std::make_unique<GameState<GamePhase::FINISHED>>(GameState<GamePhase::FINISHED>{nextHalf}));
                    break;
                }
            }
        }
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
