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
    using gamecontroller::GameControllerPacket;
    using gamecontroller::GameControllerReplyPacket;
    using gamecontroller::ReplyMessage;
    using gamecontroller::Team;
    using namespace messages::input::gameevents;
    using TeamColourEvent = messages::input::gameevents::TeamColour;

    GameController::GameController(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), socket(0) {

        powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&GameController::run), this), std::bind(std::mem_fn(&GameController::kill), this)));

        on<Trigger<Configuration<GameController>>>("GameController Configuration", [this](const Configuration<GameController>& config) {

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

        on<Trigger<Every<5, std::chrono::seconds>>>("GameController Reply", [this](const time_t&) {
            if (socket) {
                auto replyPacket = std::make_unique<GameControllerReplyPacket>();
                std::copy(std::begin(gamecontroller::RETURN_HEADER), std::end(gamecontroller::RETURN_HEADER), std::begin(replyPacket->header));
                replyPacket->version = gamecontroller::RETURN_VERSION;
                replyPacket->team = TEAM_ID;
                replyPacket->player = PLAYER_ID;
                replyPacket->message = uint8_t(ReplyMessage::UNPENALISE); // TODO
                ::send(socket, reinterpret_cast<char *>(&replyPacket), sizeof(replyPacket), 0);
            }
        });

        auto gameState = std::make_unique<GameState>();
        // default to reasonable values for initial state
        gameState->phase = Phase::INITIAL;
        gameState->mode = Mode::NORMAL;
        gameState->firstHalf = true;
        emit(std::move(gameState));

    }

    void GameController::run() {
        sockaddr_in broadcastSocket;
        GameControllerPacket newPacket;
        unsigned slen = sizeof(sockaddr);

        while (listening) {
            recvfrom(socket, reinterpret_cast<char*>(&newPacket), sizeof(GameControllerPacket), 0, reinterpret_cast<sockaddr*>(&broadcastSocket), &slen);

            if (newPacket.version == SUPPORTED_VERSION) {
                process(packet, newPacket);

                packet = newPacket;

            }
        }
    }

    void GameController::process(GameControllerPacket oldPacket, GameControllerPacket newPacket) {

        auto state = std::make_unique<GameState>(*powerplant.get<GameState>());

        std::vector<std::function<void ()>> stateChanges;

        // game score
        Team& oldOwnTeam = getOwnTeam(oldPacket);
        Team& newOwnTeam = getOwnTeam(newPacket);

        Team& oldOpponentTeam = getOpponentTeam(oldPacket);
        Team& newOpponentTeam = getOpponentTeam(newPacket);

        /*******************************************************************************************
         * Process score updates
         ******************************************************************************************/
        if (oldOwnTeam.score != newOwnTeam.score || oldOpponentTeam.score != newOpponentTeam.score) {
            // score update
            emit(std::make_unique<Score>(Score{newOwnTeam.score, newOpponentTeam.score}));


            if (oldOwnTeam.score > newOwnTeam.score) {
                // we scored! :D

                // Set the team scores in the state packet
                state->team.score = newOwnTeam.score;
                stateChanges.push_back([this, newOwnTeam] {
                    emit(std::make_unique<GoalScored<TEAM>>(GoalScored<TEAM>{newOwnTeam.score}));
                });
            }

            if (oldOpponentTeam.score > newOpponentTeam.score) {
                // they scored :( boo

                // Set the team scores in the state packet
                state->opponent.score = newOpponentTeam.score;
                stateChanges.push_back([this, newOpponentTeam] {
                    emit(std::make_unique<GoalScored<OPPONENT>>(GoalScored<OPPONENT>{newOpponentTeam.score}));
                });
            }

        }


        /*******************************************************************************************
         * Process penality updates
         ******************************************************************************************/
        // Clear our player state (easier to just rebuild)
        state->team.players.clear();
        state->opponent.players.clear();

        // Note: assumes playersPerTeam never changes
        for (uint i = 0; i < newPacket.playersPerTeam; i++) {
            auto& oldOwnPlayer = oldOwnTeam.players[i];
            auto& newOwnPlayer = newOwnTeam.players[i];

            auto& oldOpponentPlayer = oldOpponentTeam.players[i];
            auto& newOpponentPlayer = newOpponentTeam.players[i];

            // Update our state
            state->team.players.push_back({
                newOwnPlayer.penalised,
                NUClear::clock::now() + std::chrono::seconds(newOwnPlayer.penalisedTimeLeft)
            });
            state->opponent.players.push_back({
                newOpponentPlayer.penalised,
                NUClear::clock::now() + std::chrono::seconds(newOpponentPlayer.penalisedTimeLeft)
            });

            // check if player on own team is penalised
            if (newOwnPlayer.penalised && !oldOwnPlayer.penalised) {
                auto unpenalisedTime = NUClear::clock::now() + std::chrono::seconds(newOwnPlayer.penalisedTimeLeft);
                stateChanges.push_back([this, i, unpenalisedTime] {
                    if (i == PLAYER_ID) {
                        // self penalised :@
                        emit(std::make_unique<Penalisation<SELF>>(Penalisation<SELF>{i, unpenalisedTime}));
                    }
                    else {
                        // team mate penalised :'(
                        emit(std::make_unique<Penalisation<TEAM>>(Penalisation<TEAM>{i, unpenalisedTime}));
                    }
                });
            }
            else if (!newOwnPlayer.penalised && oldOwnPlayer.penalised) {
                stateChanges.push_back([this, i] {
                    if (i == PLAYER_ID) {
                        // self unpenalised :)
                        emit(std::make_unique<Unpenalisation<SELF>>(Unpenalisation<SELF>{i}));
                    }
                    else {
                        // team mate unpenalised :)
                        emit(std::make_unique<Unpenalisation<TEAM>>(Unpenalisation<TEAM>{i}));
                    }
                });
            }

            if (newOpponentPlayer.penalised && !oldOpponentPlayer.penalised) {
                auto unpenalisedTime = NUClear::clock::now() + std::chrono::seconds(newOpponentPlayer.penalisedTimeLeft);
                // opponent penalised :D
                stateChanges.push_back([this, i, unpenalisedTime] {
                    emit(std::make_unique<Penalisation<OPPONENT>>(Penalisation<OPPONENT>{i, unpenalisedTime}));
                });
            }
            else if (!newOpponentPlayer.penalised && oldOpponentPlayer.penalised) {
                // opponent unpenalised D:
                stateChanges.push_back([this, i] {
                    emit(std::make_unique<Unpenalisation<OPPONENT>>(Unpenalisation<OPPONENT>{i}));
                });
            }
        }


        /*******************************************************************************************
         * Process coach messages
         ******************************************************************************************/
        if (std::strcmp(oldOwnTeam.coachMessage, newOwnTeam.coachMessage) != 0) {

            // Update thhe coach message in the state
            state->team.coachMessage = newOwnTeam.coachMessage;

            // listen to the coach? o_O
            stateChanges.push_back([this, newOwnTeam] {
                emit(std::make_unique<CoachMessage<TEAM>>(CoachMessage<TEAM>{newOwnTeam.coachMessage}));
            });
        }

        if (std::strcmp(oldOpponentTeam.coachMessage, newOpponentTeam.coachMessage) != 0) {

            // Update the opponent coach message in the state
            state->opponent.coachMessage = newOpponentTeam.coachMessage;

            // listen in on the enemy! >:D
            stateChanges.push_back([this, newOpponentTeam] {
                emit(std::make_unique<CoachMessage<OPPONENT>>(CoachMessage<OPPONENT>{newOpponentTeam.coachMessage}));
            });
        }


        /*******************************************************************************************
         * Process half changes
         ******************************************************************************************/
        if (oldPacket.firstHalf != newPacket.firstHalf) {

            // Update the half time in the state
            state->firstHalf = newPacket.firstHalf;

            // half time
            stateChanges.push_back([this, newPacket] {
                emit(std::make_unique<HalfTime>(HalfTime{newPacket.firstHalf}));
            });
        }

        /*******************************************************************************************
         * Process ball kicked out
         ******************************************************************************************/
        if (oldPacket.dropInTime >= 0 && oldPacket.dropInTime > newPacket.dropInTime)  {

            // ball was kicked out by dropInTeam
            auto time = NUClear::clock::now() - std::chrono::seconds(newPacket.dropInTime);

            // Update the ball kicked out time and player in the state
            state->kickedOutByUs = newPacket.dropInTeam == newOwnTeam.teamColour;
            state->kickedOutTime = time;

            if (newPacket.dropInTeam == newOwnTeam.teamColour) {
                // we kicked the ball out :S
                stateChanges.push_back([this, time] {
                    emit(std::make_unique<BallKickedOut<TEAM>>(BallKickedOut<TEAM>{time}));
                });
            }
            else {
                // they kicked the ball out! ^_^
                stateChanges.push_back([this, time] {
                    emit(std::make_unique<BallKickedOut<OPPONENT>>(BallKickedOut<OPPONENT>{time}));
                });
            }
        }


        /*******************************************************************************************
         * Process kick off team
         ******************************************************************************************/
        if (oldPacket.kickOffTeam != newPacket.kickOffTeam) {

            // Update the kickoff team (us or them)
            state->ourKickOff = newPacket.kickOffTeam == newOwnTeam.teamColour;

            // new kick off team? :/
            Context team = newPacket.kickOffTeam == newOwnTeam.teamColour ? TEAM : OPPONENT;
            stateChanges.push_back([this, team] {
                emit(std::make_unique<KickOffTeam>(KickOffTeam{team}));
            });
        }


        /*******************************************************************************************
         * Process our team colour
         ******************************************************************************************/
        if (oldOwnTeam.teamColour != newOwnTeam.teamColour) {
            TeamColour colour = newOwnTeam.teamColour == gamecontroller::TeamColour::CYAN ? TeamColour::CYAN : TeamColour::MAGENTA;
            stateChanges.push_back([this, colour] {
                emit(std::make_unique<TeamColour>(colour));
            });
        }


        /*******************************************************************************************
         * Process state/mode changes
         ******************************************************************************************/
        if (oldPacket.mode != newPacket.mode && newPacket.mode != gamecontroller::Mode::TIMEOUT) {

            // TODO update our mode in our state

            // Changed modes but not to timeout
            switch (newPacket.mode) {
                case gamecontroller::Mode::NORMAL:
                    state->mode = Mode::NORMAL;
                    stateChanges.push_back([this] {
                        emit(std::make_unique<GameMode<Mode::NORMAL>>());
                    });
                    break;
                case gamecontroller::Mode::PENALTY_SHOOTOUT:
                    state->mode = Mode::PENALTY_SHOOTOUT;
                    stateChanges.push_back([this] {
                        emit(std::make_unique<GameMode<Mode::PENALTY_SHOOTOUT>>());
                    });
                    break;
                case gamecontroller::Mode::OVERTIME:
                    state->mode = Mode::OVERTIME;
                    stateChanges.push_back([this] {
                        emit(std::make_unique<GameMode<Mode::OVERTIME>>());
                    });
                    break;
                default:
                    throw std::runtime_error("Invalid mode change");
            }
        }

        if (oldPacket.mode != gamecontroller::Mode::TIMEOUT && newPacket.mode == gamecontroller::Mode::TIMEOUT) {

            // Change the game state to timeout
            auto time = NUClear::clock::now() + std::chrono::seconds(newPacket.secondaryTime);

            state->phase = Phase::TIMEOUT;
            state->secondaryTime = time;

            stateChanges.push_back([this, time] {
                emit(std::make_unique<GamePhase<Phase::TIMEOUT>>(GamePhase<Phase::TIMEOUT>{time}));
            });
        }
        else if (oldPacket.state != newPacket.state) {

            // State has changed, process it

            switch (newPacket.state) {
                case gamecontroller::State::INITIAL: {

                    state->phase = Phase::INITIAL;

                    stateChanges.push_back([this] {
                        emit(std::make_unique<GamePhase<Phase::INITIAL>>());
                    });
                    break;
                }
                case gamecontroller::State::READY: {
                    /* Note: It was concluded that a specific 'dropped ball' event
                     * was not needed, but in the case that it is, this is how and
                     * where you would emit it:
                     */
                    // if (oldPacket.state == State::PLAYING) {
                    //     // DROPPED BALL
                    // }
                    auto time = NUClear::clock::now() + std::chrono::seconds(newPacket.secondaryTime);

                    state->phase = Phase::READY;
                    state->secondaryTime = time;

                    stateChanges.push_back([this, time] {
                        emit(std::make_unique<GamePhase<Phase::READY>>(GamePhase<Phase::READY>{time}));
                    });
                    break;
                }
                case gamecontroller::State::SET: {

                    state->phase = Phase::READY;

                    stateChanges.push_back([this] {
                        emit(std::make_unique<GamePhase<Phase::SET>>());
                    });
                    break;
                }
                case gamecontroller::State::PLAYING: {
                    auto endHalf = NUClear::clock::now() + std::chrono::seconds(newPacket.secsRemaining);
                    auto ballFree = NUClear::clock::now() + std::chrono::seconds(newPacket.secondaryTime);

                    state->primaryTime = endHalf;
                    state->secondaryTime = ballFree;
                    state->phase = Phase::PLAYING;

                    stateChanges.push_back([this, endHalf, ballFree] {
                        emit(std::make_unique<GamePhase<Phase::PLAYING>>(GamePhase<Phase::PLAYING>{endHalf, ballFree}));
                    });
                    break;
                }
                case gamecontroller::State::FINISHED: {

                    auto nextHalf = NUClear::clock::now() + std::chrono::seconds(newPacket.secsRemaining);

                    state->primaryTime = nextHalf;
                    state->phase = Phase::FINISHED;

                    stateChanges.push_back([this, nextHalf] {
                        emit(std::make_unique<GamePhase<Phase::FINISHED>>(GamePhase<Phase::FINISHED>{nextHalf}));
                    });
                    break;
                }
            }

        }

        if (!stateChanges.empty()) {
            // emit new state
            emit(std::move(state));

            // emit individual state change events
            for (auto& change : stateChanges) {
                change();
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
