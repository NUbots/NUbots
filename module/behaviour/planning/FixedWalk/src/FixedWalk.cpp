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

#include "FixedWalk.h"

#include "message/motion/GetupCommand.h"
#include "utility/math/matrix/Rotation2D.h"

namespace module {
namespace behaviour {
    namespace planning {

        using message::behaviour::CancelFixedWalk;
        using message::behaviour::FixedWalkCommand;
        using message::behaviour::FixedWalkFinished;
        using message::input::Sensors;
        using message::motion::ExecuteGetup;
        using message::motion::KillGetup;
        using message::motion::StopCommand;
        using message::motion::WalkCommand;
        using message::motion::WalkStopped;
        using utility::math::matrix::Rotation2D;


        FixedWalk::FixedWalk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), active(false) {

            on<Trigger<ExecuteGetup>>().then("FixedWalk::Getup", [this] {
                // record fall time
                segmentElapsedTimeBeforeFall = NUClear::clock::now() - segmentStart;
                fallen                       = true;
            });

            on<Trigger<KillGetup>>().then("FixedWalk::Getup Finished", [this] {
                // getup finished
                segmentStart = NUClear::clock::now() - segmentElapsedTimeBeforeFall;
                fallen       = false;
            });

            on<Every<30, Per<std::chrono::seconds>>, With<Sensors>, Sync<FixedWalk>>().then(
                "Fixed Walk Manager", [this](const Sensors& sensors) {
                    if (active && t > segmentStart + walkSegments.front().duration && !fallen) {
                        // Move to next segment
                        segmentStart += walkSegments.front().duration;
                        walkSegments.pop_front();

                        if (walkSegments.empty()) {
                            emit(std::make_unique<WalkCommand>());
                            emit(std::make_unique<StopCommand>());
                            active = false;
                            return;
                        }
                        beginningOrientation = sensors.world.rotation();
                    }
                    // Emit command
                    if (!walkSegments.empty()) {
                        emit(getWalkCommand(walkSegments.front(), t - segmentStart, sensors));
                    }
                });

            on<Trigger<CancelFixedWalk>, Sync<FixedWalk>>().then([this] {
                emit(std::make_unique<StopCommand>());
                active = false;
                walkSegments.clear();
            });

            on<Trigger<WalkStopped>, Sync<FixedWalk>>().then([this] {
                if (!active) {
                    emit(std::make_unique<FixedWalkFinished>());
                }
                else {
                    NUClear::log("!!!!!!!!!!!!!!!!!!!!WARNING: Walk finished prematurely!!!!!!!!!!!!!!!!!!!!");
                }
            });

            on<Trigger<FixedWalkCommand>, With<Sensors>, Sync<FixedWalk>>().then(
                [this](const FixedWalkCommand& command, const Sensors& sensors) {
                    if (!active && !command.segments.empty()) {
                        active               = true;
                        segmentStart         = NUClear::clock::now();
                        beginningOrientation = sensors.world.rotation();
                    }
                    for (auto& segment : command.segments) {
                        walkSegments.push_back(segment);
                    }
                    log("FixedWalk::FixedWalkCommand - Total walk segments pushed back:",
                        walkSegments.size(),
                        command.segments.size());
                });
        }

        std::unique_ptr<WalkCommand> FixedWalk::getWalkCommand(const FixedWalkCommand::WalkSegment& segment,
                                                               NUClear::clock::duration t,
                                                               const Sensors&) {
            double timeSeconds = std::chrono::duration_cast<std::chrono::seconds>(t).count();
            arma::vec2 directionInOriginalCoords =
                (segment.curvePeriod != 0 ? Rotation2D::createRotation(2 * M_PI * timeSeconds / segment.curvePeriod)
                                          : arma::eye(2, 2))
                * segment.direction;
            arma::vec2 direction    = arma::normalise(directionInOriginalCoords);
            auto result             = std::make_unique<WalkCommand>();
            result->command.xy()    = segment.normalisedVelocity * direction;
            result->command.angle() = segment.normalisedAngularVelocity;
            return std::move(result);
        }


    }  // namespace planning
}  // namespace behaviour
}  // namespace module
