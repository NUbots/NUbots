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

#include "SoccerStrategy.h"
#include "messages/input/gameevents/GameEvents.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/behaviour/LookStrategy.h"
#include "messages/behaviour/WalkStrategy.h"

namespace modules {
namespace behaviour {
namespace strategy {
    using messages::input::gameevents::GameState;
    using messages::input::gameevents::Mode;
    using messages::input::gameevents::Phase;
    using messages::platform::darwin::ButtonLeftDown;
    using messages::platform::darwin::ButtonMiddleDown;
    using messages::behaviour::WalkStrategy;
    using messages::behaviour::WalkApproach;
    using messages::behaviour::FieldTarget;

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // For checking last seen times
        on<Trigger<std::vector<VisionBall>>>([this] (const std::vector<VisionBall>& balls) {
            if(!balls.empty()) {
                ballLastSeen = NUClear::clock::now();
            }
        });

        // Main Loop
        on<Trigger<Every<30, Per<std::chrono::seconds>>>, With<GameState>, // TODO: ensure a reasonable state is emitted even if gamecontroller is not running
            Options<Single>>([this](const time_t&, const GameState& gameState) {

            auto& mode = gameState.mode;
            auto& phase = gameState.phase;

            if (pickedUp()) {
                // TODO: stand, no moving
            }
            else {
                if (mode == Mode::NORMAL || mode == Mode::OVERTIME) {
                    if (phase == Phase::INITIAL) {
                        standStill();
                        find({FieldTarget::SELF});
                    }
                    else if (phase == Phase::READY) {
                        // TODO: walkTo(myZone.startPosition);
                        find({FieldTarget::SELF});
                    }
                    else if (phase == Phase::SET) {
                        standStill();
                        find({FieldTarget::BALL});
                    }
                    else if (phase == Phase::TIMEOUT) {
                        standStill();
                        find({FieldTarget::SELF});
                    }
                    else if (phase == Phase::FINISHED) {
                        standStill();
                        find({FieldTarget::SELF});
                    }
                    else if (phase == Phase::PLAYING) {
                        if (penalised()) { // penalised
                            standStill();
                            find({FieldTarget::SELF});
                        }
                        else { // not penalised
                            if (NUClear::clock::now() - ballLastSeen < zone.ballActiveTimeout) { // ball has been seen recently
                                if (inZone(FieldTarget::BALL) || isClose(FieldTarget::BALL)) { // in zone and close to ball
                                    // TODO: walkTo(FieldTarget::BALL);
                                    find({FieldTarget::BALL});
                                }
                                else { // not in zone and not close to ball
                                    if (isGoalie()) { // goalie
                                        // TODO: walkTo(myZone.defaultPosition, Align::OPPOSITION_GOAL});
                                        find({FieldTarget::BALL});
                                    }
                                    else { // not goalie
                                        // TODO: walkTo(myZone.defaultPosition);
                                        find({FieldTarget::BALL});
                                    }
                                }
                            }
                            else { // ball has not been seen recently
                                if (inZone(FieldTarget::SELF)) { // in own zone
                                    // TODO: walkTo(myZone.defaultPosition);
                                    find({FieldTarget::SELF, FieldTarget::BALL});
                                }
                                else { // not in zone
                                    if (NUClear::clock::now() - ballLastSeen < zone.zoneReturnTimeout) {
                                        spinWalk();
                                        find({FieldTarget::BALL});
                                    }
                                    else { // time passed since ball seen
                                        // TODO: walkTo(myZone.defaultPosition);
                                        find({FieldTarget::SELF, FieldTarget::BALL});
                                    }

                                }
                            }
                        }
                    }
                }
                else if (mode == Mode::PENALTY_SHOOTOUT) {

                }
            }
        });
    }

    void SoccerStrategy::searchWalk() {

    }

    void SoccerStrategy::standStill() {
        auto command = std::make_unique<WalkStrategy>();
        command->walkMovementType = WalkApproach::StandStill;
        emit(std::move(command));
    }

    void SoccerStrategy::walkTo(const FieldTarget& object) {
        // TODO: find object position and call other walkTo method
    }

    void SoccerStrategy::walkTo(arma::vec position) {
        // TODO: Get the position of the target
        // Send this to the walk planner
    }

    bool SoccerStrategy::pickedUp() {
        // TODO: currentState.pickedUp = feetOffGround && !isGettingUp && !isDiving;
        return false; // TODO
    }

    bool SoccerStrategy::penalised() {
        // TODO: Get our game state packet and return if we are penalised
        return false; // TODO
    }

    bool SoccerStrategy::recentlyVisible(const FieldTarget& object) {
        // TODO: Look at the last timestamp of the seen object (using triggers for caching the timestamps)
        return true; // TODO
    }

    bool SoccerStrategy::isGoalie() {
        // TODO: checking myzone.isgoalie true
        return false; // TODO
    }

    // template <typename T>
    // bool SoccerStrategy::isClose() {
    bool SoccerStrategy::isClose(const FieldTarget& object) {
        // auto fieldObject = powerplant.get<T>();
        // obj.position LFiajfoijsafoisje
        // TODO: use old behaviour code
        return true; // TODO
    }

    bool SoccerStrategy::inZone(const FieldTarget& object) {
        // TODO: check that object is in bounding box
        return true; // TODO
    }

    void SoccerStrategy::find(const std::vector<FieldTarget>& fieldObjects) {
        // TODO:
    }

    bool SoccerStrategy::timePassed() {
        // TODO: use recently seen instead of this
        return false;
    }

    void SoccerStrategy::spinWalk() {
        // TODO: undefined, maybe send a custom walk command?
    }



}  // strategy
}  // behaviours
} // modules
