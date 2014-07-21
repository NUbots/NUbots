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
#include "messages/support/Configuration.h"
#include "messages/support/GlobalConfig.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/output/Say.h"

extern "C" {
    #include <sys/socket.h>
    #include <arpa/inet.h>
}

namespace modules {
namespace input {

    using messages::support::Configuration;
    using messages::support::GlobalConfig;
    using gamecontroller::GameControllerPacket;
    using gamecontroller::GameControllerReplyPacket;
    using gamecontroller::ReplyMessage;
    using gamecontroller::Team;
    using namespace messages::input::gameevents;
    using TeamColourEvent = messages::input::gameevents::TeamColour;
    using messages::platform::darwin::ButtonLeftDown;
    using messages::platform::darwin::ButtonMiddleDown;

    GameController::GameController(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), socket(0) {

        powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&GameController::run), this), std::bind(std::mem_fn(&GameController::kill), this)));

        auto setup = [this] (const Configuration<GameController>& config, const GlobalConfig& globalConfig) {

            // TODO use an eventfd to allow changing the port dynamically

            PLAYER_ID = globalConfig.playerId;
            TEAM_ID = globalConfig.teamId;
            BROADCAST_IP = config["broadcastIP"].as<std::string>();

            port = config["port"].as<uint>();
            sockaddr_in socketAddress;
            memset(&socketAddress, 0, sizeof(socketAddress));
            int broadcast = 1;
            socketAddress.sin_family = AF_INET;
            socketAddress.sin_port = htons(port);
            socketAddress.sin_addr.s_addr = (BROADCAST_IP.empty() ? INADDR_ANY : inet_addr(BROADCAST_IP.c_str()));

            int newSocket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            setsockopt(newSocket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
            ::bind(newSocket, reinterpret_cast<sockaddr*>(&socketAddress), sizeof(sockaddr));

            int oldSocket = socket.exchange(newSocket);

            if (oldSocket) {
                ::close(oldSocket);
            }

            mode = static_cast<gamecontroller::Mode>(-1);

            std::copy(std::begin(gamecontroller::RECEIVE_HEADER), std::end(gamecontroller::RECEIVE_HEADER), std::begin(packet.header));
            packet.version = SUPPORTED_VERSION;
            packet.packetNumber = 0;
            packet.playersPerTeam = PLAYERS_PER_TEAM;
            packet.state = static_cast<gamecontroller::State>(-1);
            packet.firstHalf = true;
            packet.kickOffTeam = static_cast<gamecontroller::TeamColour>(-1);
            packet.mode = static_cast<gamecontroller::Mode>(-1);
            packet.dropInTeam = static_cast<gamecontroller::TeamColour>(-1);
            packet.dropInTime = -1;
            packet.secsRemaining = 0;
            packet.secondaryTime = 0;
            for (uint i = 0; i < NUM_TEAMS; i++) {
                auto& ownTeam = packet.teams[i];
                ownTeam.teamId = (i == 0 ? TEAM_ID : 0);
                ownTeam.teamColour = static_cast<gamecontroller::TeamColour>(-1);
                ownTeam.score = 0;
                ownTeam.penaltyShot = 0;
                ownTeam.singleShots = 0;
                ownTeam.coachMessage.fill(0);
                auto& coach = ownTeam.coach;
                coach.penaltyState = gamecontroller::PenaltyState::UNPENALISED;
                coach.penalisedTimeLeft = 0;
                for (uint i = 0; i < gamecontroller::MAX_NUM_PLAYERS; i++) {
                    auto& player = ownTeam.players[i];
                    if (i <= ACTIVE_PLAYERS_PER_TEAM) {
                        player.penaltyState = gamecontroller::PenaltyState::UNPENALISED;
                        player.penalisedTimeLeft = 0;
                    } else {
                        player.penaltyState = gamecontroller::PenaltyState::SUBSTITUTE;
                        player.penalisedTimeLeft = 0;
                    }
                }
            }

            auto initialState = std::make_unique<GameState>();
            // default to reasonable values for initial state
            initialState->phase = Phase::INITIAL;
            initialState->mode = Mode::NORMAL;
            initialState->firstHalf = true;
            initialState->kickedOutByUs = false;
            initialState->ourKickOff = false;

            initialState->team.teamId = TEAM_ID;
            initialState->opponent.teamId = 0;

            emit(std::move(initialState));
            emit(std::make_unique<Phase>(Phase::INITIAL));
        };

        // Trigger the same function when either update
        on<With<Configuration<GameController>>, Trigger<GlobalConfig>>("GameController Configuration", setup);
        on<Trigger<Configuration<GameController>>, With<GlobalConfig>>("GameController Configuration", setup);

        on<Trigger<Every<2, Per<std::chrono::seconds>>>>("GameController Reply", [this](const time_t&) {
            sendReplyPacket(ReplyMessage::ALIVE);
        });

        on<Trigger<ButtonLeftDown>>([this](const ButtonLeftDown&) {
            // TODO: aggressive mode, chase ball and kick towards goal (basically disable strategy)
        });

        on<Trigger<ButtonMiddleDown>>([this](const ButtonMiddleDown&) {
            // TODO: fix this
            auto time = NUClear::clock::now();
            if (!selfPenalised) {
                // penalise
                selfPenalised = true;
                emit(std::move(std::make_unique<messages::output::Say>("Penalised")));
                emit(std::make_unique<Penalisation<SELF>>(Penalisation<SELF>{PLAYER_ID, time, PenaltyReason::MANUAL}));
            } else {
                // unpenalised
                selfPenalised = false;
                emit(std::move(std::make_unique<messages::output::Say>("Unpenalised")));
                emit(std::make_unique<Unpenalisation<SELF>>(Unpenalisation<SELF>{PLAYER_ID}));
                emit(std::make_unique<Penalisation<SELF>>(Penalisation<SELF>{PLAYER_ID, time, PenaltyReason::MANUAL}));
                // TODO: fix timers
                emit(std::make_unique<GamePhase<Phase::PLAYING>>(GamePhase<Phase::PLAYING>{time, time}));
                emit(std::make_unique<Phase>(Phase::PLAYING));
            }
            penaltyOverride = true;
        });

    }

    void GameController::sendReplyPacket(const ReplyMessage& replyMessage) const {
        if (socket) {

            sockaddr_in socketAddress;
            memset(&socketAddress, 0, sizeof(socketAddress));
            socketAddress.sin_family = AF_INET;
            socketAddress.sin_port = htons(port);
            socketAddress.sin_addr.s_addr = (BROADCAST_IP.empty() ? INADDR_ANY : inet_addr(BROADCAST_IP.c_str()));

            GameControllerReplyPacket replyPacket;
            std::copy(std::begin(gamecontroller::RETURN_HEADER), std::end(gamecontroller::RETURN_HEADER), std::begin(replyPacket.header));
            replyPacket.version = gamecontroller::RETURN_VERSION;
            replyPacket.team = TEAM_ID;
            replyPacket.player = PLAYER_ID;
            replyPacket.message = replyMessage;
            if(::sendto(socket, reinterpret_cast<char*>(&replyPacket), sizeof(replyPacket), 0, reinterpret_cast<sockaddr*>(&socketAddress), sizeof(socketAddress)) < 0) {
                throw std::system_error(errno, std::system_category());
            }
        }
    }

    void GameController::run() {
        GameControllerPacket newPacket;

        while (listening) {
            ::recv(socket, reinterpret_cast<char*>(&newPacket), sizeof(GameControllerPacket), 0);

            if (newPacket.version == SUPPORTED_VERSION) {
                try {
                    process(packet, newPacket);
                } catch (std::runtime_error err) {
                    log(err.what());
                    continue;
                }

                packet = newPacket;

            }
        }
    }

    void GameController::process(const GameControllerPacket& oldPacket, const GameControllerPacket& newPacket) {

        auto state = std::make_unique<GameState>(*powerplant.get<GameState>());

        std::vector<std::function<void ()>> stateChanges;

        // game score
        auto& oldOwnTeam = getOwnTeam(oldPacket);
        auto& newOwnTeam = getOwnTeam(newPacket);

        auto& oldOpponentTeam = getOpponentTeam(oldPacket);
        auto& newOpponentTeam = getOpponentTeam(newPacket);

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
         * Process penalty updates
         ******************************************************************************************/
        // Clear our player state (easier to just rebuild)
        state->team.players.clear();
        state->opponent.players.clear();

        // Note: assumes playersPerTeam never changes
        for (uint i = 0; i < newPacket.playersPerTeam; i++) {
            uint playerId = i + 1;
            auto& oldOwnPlayer = oldOwnTeam.players[i];
            auto& newOwnPlayer = newOwnTeam.players[i];

            auto& oldOpponentPlayer = oldOpponentTeam.players[i];
            auto& newOpponentPlayer = newOpponentTeam.players[i];

            // Update our state
            state->team.players.push_back({
                playerId,
                getPenaltyReason(newOwnPlayer.penaltyState),
                NUClear::clock::now() + std::chrono::seconds(newOwnPlayer.penalisedTimeLeft)
            });
            state->opponent.players.push_back({
                playerId,
                getPenaltyReason(newOpponentPlayer.penaltyState),
                NUClear::clock::now() + std::chrono::seconds(newOpponentPlayer.penalisedTimeLeft)
            });

            // check if player on own team is penalised
            if (newOwnPlayer.penaltyState != oldOwnPlayer.penaltyState
                && newOwnPlayer.penaltyState != gamecontroller::PenaltyState::UNPENALISED) {

                auto unpenalisedTime = NUClear::clock::now() + std::chrono::seconds(newOwnPlayer.penalisedTimeLeft);
                auto reason = getPenaltyReason(newOwnPlayer.penaltyState);
                stateChanges.push_back([this, playerId, unpenalisedTime, reason] {
                    if (playerId == PLAYER_ID) {
                        // self penalised :@
                        emit(std::make_unique<Penalisation<SELF>>(Penalisation<SELF>{playerId, unpenalisedTime, reason}));
                        sendReplyPacket(ReplyMessage::PENALISED);
                        selfPenalised = true;
                        penaltyOverride = false;
                    }
                    else {
                        // team mate penalised :'(
                        emit(std::make_unique<Penalisation<TEAM>>(Penalisation<TEAM>{playerId, unpenalisedTime, reason}));
                    }
                });
            }
            else if (newOwnPlayer.penaltyState == gamecontroller::PenaltyState::UNPENALISED
                && oldOwnPlayer.penaltyState != gamecontroller::PenaltyState::UNPENALISED) {
                stateChanges.push_back([this, playerId] {
                    if (playerId == PLAYER_ID) {
                        // self unpenalised :)
                        emit(std::make_unique<Unpenalisation<SELF>>(Unpenalisation<SELF>{playerId}));
                        sendReplyPacket(ReplyMessage::UNPENALISED);
                        selfPenalised = false;
                        penaltyOverride = false;
                    }
                    else {
                        // team mate unpenalised :)
                        emit(std::make_unique<Unpenalisation<TEAM>>(Unpenalisation<TEAM>{playerId}));
                    }
                });
            }

            if (newOpponentPlayer.penaltyState != oldOpponentPlayer.penaltyState
                && newOpponentPlayer.penaltyState != gamecontroller::PenaltyState::UNPENALISED) {
                auto unpenalisedTime = NUClear::clock::now() + std::chrono::seconds(newOpponentPlayer.penalisedTimeLeft);
                auto reason = getPenaltyReason(newOpponentPlayer.penaltyState);
                // opponent penalised :D
                stateChanges.push_back([this, playerId, unpenalisedTime, reason] {
                    emit(std::make_unique<Penalisation<OPPONENT>>(Penalisation<OPPONENT>{playerId, unpenalisedTime, reason}));
                });
            }
            else if (newOpponentPlayer.penaltyState == gamecontroller::PenaltyState::UNPENALISED
                && oldOpponentPlayer.penaltyState != gamecontroller::PenaltyState::UNPENALISED) {
                // opponent unpenalised D:
                stateChanges.push_back([this, playerId] {
                    emit(std::make_unique<Unpenalisation<OPPONENT>>(Unpenalisation<OPPONENT>{playerId}));
                });
            }
        }


        /*******************************************************************************************
         * Process coach messages
         ******************************************************************************************/
        if (oldOwnTeam.coachMessage != newOwnTeam.coachMessage) {

            // Update the coach message in the state
            state->team.coachMessage = newOwnTeam.coachMessage.data();

            // listen to the coach? o_O
            stateChanges.push_back([this, newOwnTeam] {
                emit(std::make_unique<CoachMessage<TEAM>>(CoachMessage<TEAM>{newOwnTeam.coachMessage.data()}));
            });
        }

        if (oldOpponentTeam.coachMessage != newOpponentTeam.coachMessage) {

            // Update the opponent coach message in the state
            state->opponent.coachMessage = newOpponentTeam.coachMessage.data();

            // listen in on the enemy! >:D
            stateChanges.push_back([this, newOpponentTeam] {
                emit(std::make_unique<CoachMessage<OPPONENT>>(CoachMessage<OPPONENT>{newOpponentTeam.coachMessage.data()}));
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
        // Woo boolean algebra!
        if ((newPacket.dropInTime != -1 && ((newPacket.dropInTime < oldPacket.dropInTime) == (oldPacket.dropInTime != -1))) || newPacket.dropInTeam != oldPacket.dropInTeam) {

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
        if (newPacket.mode != mode
            && oldPacket.mode != gamecontroller::Mode::TIMEOUT
            && newPacket.mode != gamecontroller::Mode::TIMEOUT) {

            mode = newPacket.mode;

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
                emit(std::make_unique<Mode>(state->mode));
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
        else if (oldPacket.state != newPacket.state || (oldPacket.mode == gamecontroller::Mode::TIMEOUT && newPacket.mode != gamecontroller::Mode::TIMEOUT)) {

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

                    state->phase = Phase::SET;

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

            emit(std::make_unique<Phase>(state->phase));

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

    PenaltyReason GameController::getPenaltyReason(const gamecontroller::PenaltyState& penaltyState) const {
        // ugly incoming
        switch (penaltyState) {
            case gamecontroller::PenaltyState::UNPENALISED:
                return PenaltyReason::UNPENALISED;
            case gamecontroller::PenaltyState::BALL_MANIPULATION:
                return PenaltyReason::BALL_MANIPULATION;
            case gamecontroller::PenaltyState::PHYSICAL_CONTACT:
                return PenaltyReason::PHYSICAL_CONTACT;
            case gamecontroller::PenaltyState::ILLEGAL_ATTACK:
                return PenaltyReason::ILLEGAL_ATTACK;
            case gamecontroller::PenaltyState::ILLEGAL_DEFENSE:
                return PenaltyReason::ILLEGAL_DEFENSE;
            case gamecontroller::PenaltyState::REQUEST_FOR_PICKUP:
                return PenaltyReason::REQUEST_FOR_PICKUP;
            case gamecontroller::PenaltyState::REQUEST_FOR_SERVICE:
                return PenaltyReason::REQUEST_FOR_SERVICE;
            case gamecontroller::PenaltyState::REQUEST_FOR_PICKUP_TO_SERVICE:
                return PenaltyReason::REQUEST_FOR_PICKUP_TO_SERVICE;
            case gamecontroller::PenaltyState::SUBSTITUTE:
                return PenaltyReason::SUBSTITUTE;
            case gamecontroller::PenaltyState::MANUAL:
                return PenaltyReason::MANUAL;
            default:
                throw std::runtime_error("Invalid Penalty State");
        }
    }

    const Team& GameController::getOwnTeam(const GameControllerPacket& state) const {

        for (auto& team : state.teams) {
            // std::cout << uint(team.teamId) << ", " << uint(TEAM_ID) << std::endl;
            if (team.teamId == TEAM_ID) {
                return team;
            }
        }

        throw std::runtime_error("Own team not found");
    }

    const Team& GameController::getOpponentTeam(const GameControllerPacket& state) const {
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
