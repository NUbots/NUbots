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

    SoccerStrategy::SoccerStrategy(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Trigger<ButtonLeftDown>>([this](const ButtonLeftDown&) {
            // TODO: aggressive mode, chase ball and kick towards goal (basically disable strategy)
        });

        on<Trigger<ButtonMiddleDown>>([this](const ButtonMiddleDown&) {
            // TODO: toggle penalised (override gamecontroller?)
            // TODO Send output to game controller
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
                        find({FieldObject::SELF});
                    }
                    else if (phase == Phase::READY) {
                        // TODO: walkTo(myZone.startPosition);
                        find({FieldObject::SELF});
                    }
                    else if (phase == Phase::SET) {
                        standStill();
                        find({FieldObject::BALL});
                    }
                    else if (phase == Phase::TIMEOUT) {
                        standStill();
                        find({FieldObject::SELF});
                    }
                    else if (phase == Phase::FINISHED) {
                        standStill();
                        find({FieldObject::SELF});
                    }
                    else if (phase == Phase::PLAYING) {
                        if (penalised()) { // penalised
                            standStill();
                            find({FieldObject::SELF});
                        }
                        else { // not penalised
                            if (recentlyVisible(FieldObject::BALL)) { // ball has been seen recently
                                if (inZone(FieldObject::BALL) || isClose(FieldObject::BALL)) { // in zone and close to ball
                                    // TODO: walkTo(FieldObject::BALL);
                                    find({FieldObject::BALL});
                                }
                                else { // not in zone and not close to ball
                                    if (isGoalie()) { // goalie
                                        // TODO: walkTo(myZone.defaultPosition, Align::OPPOSITION_GOAL);
                                        lookAt(FieldObject::BALL);
                                    }
                                    else { // not goalie
                                        // TODO: walkTo(myZone.defaultPosition);
                                        lookAt(FieldObject::BALL);
                                    }
                                }
                            }
                            else { // ball has not been seen recently
                                if (inZone(FieldObject::SELF)) { // in own zone
                                    // TODO: walkTo(myZone.defaultPosition);
                                    find({FieldObject::SELF, FieldObject::BALL});
                                }
                                else { // not in zone
                                    if (!timePassed()) {
                                        spinWalk();
                                        find({FieldObject::BALL});
                                    }
                                    else { // time passed since ball seen
                                        // TODO: walkTo(myZone.defaultPosition);
                                        find({FieldObject::SELF, FieldObject::BALL});
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

    void SoccerStrategy::walkTo(const FieldObject& object) {
        // TODO: find object position and call other walkTo method
    }

    void SoccerStrategy::walkTo(arma::vec position) {
        // Get the position of the target
        // Send this to the walk planner
    }

    bool SoccerStrategy::pickedUp() {
        return false; // TODO
    }

    bool SoccerStrategy::penalised() {
        // Get our game state packet and return if we are penalised
        return false; // TODO
    }

    bool SoccerStrategy::recentlyVisible(const FieldObject& object) {
        // Get the last
        return true; // TODO
    }

    bool SoccerStrategy::visible(const FieldObject& object) {
        return true; // TODO
    }

    bool SoccerStrategy::isGoalie() {
        return false; // TODO
    }

    // template <typename T>
    // bool SoccerStrategy::isClose() {
    bool SoccerStrategy::isClose(const FieldObject& object) {
        // auto fieldObject = powerplant.get<T>();
        // obj.position LFiajfoijsafoisje
        return true; // TODO
    }

    bool SoccerStrategy::inZone(const FieldObject& object) {
        return true; // TODO
    }

    void SoccerStrategy::lookAt(const FieldObject& object) {

    }

    void SoccerStrategy::find(const std::vector<FieldObject>& fieldObjects) {

    }

    bool SoccerStrategy::timePassed() {
        return false;
    }

    void SoccerStrategy::spinWalk() {

    }



}  // strategy
}  // behaviours
} // modules
