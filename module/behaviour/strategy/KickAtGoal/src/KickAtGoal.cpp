/*
 * This file is part of NUbots Codebase.
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

#include "KickAtGoal.hpp"

#include <Eigen/Core>

#include "extension/Configuration.hpp"

#include "message/behaviour/KickPlan.hpp"
#include "message/behaviour/MotionCommand.hpp"
#include "message/vision/Ball.hpp"
#include "message/vision/Goal.hpp"

#include "utility/behaviour/MotionCommand.h"

namespace module {
namespace behaviour {
    namespace strategy {

        using extension::Configuration;

        using message::behaviour::Behaviour;
        using message::behaviour::KickPlan;
        using message::behaviour::MotionCommand;

        using message::vision::Balls;
        using message::vision::Goals;

        NUClear::clock::duration KickAtGoal::durationFromSeconds(const double& seconds) {
            return NUClear::clock::duration(uint64_t(NUClear::clock::period::den * seconds));
        }

        KickAtGoal::KickAtGoal(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

            // TODO: unhack?
            emit(std::make_unique<KickPlan>(KickPlan(Eigen::Vector2d(3.0, 0.0), KickPlan::KickType::SCRIPTED)));

            on<Every<30, Per<std::chrono::seconds>>, Single>().then([this] { doBehaviour(); });

            on<Trigger<Balls>>().then([this](const Balls& balls) {
                if (!balls.balls.empty()) {
                    ballLastSeen = NUClear::clock::now();
                }
            });

            on<Trigger<Goals>>().then([this](const Goals& goals) {
                if (!goals.goals.empty()) {
                    goalLastSeen = NUClear::clock::now();
                }
            });

            on<Configuration>("KickAtGoal.yaml").then([this](const Configuration& config) {
                ballActiveTimeout = durationFromSeconds(config["ball_active_timeout"].as<double>());
            });
        }

        void KickAtGoal::doBehaviour() {
            // Store the state before executing behaviour.
            Behaviour::State previousState = currentState;

            // Check if the ball  has been seen recently.
            if (NUClear::clock::now() - ballLastSeen < ballActiveTimeout) {
                walkToBall();
            }
            else {
                spinToWin();
            }

            if (currentState != previousState) {
                emit(std::make_unique<Behaviour::State>(currentState));
            }
        }

        void KickAtGoal::walkToBall() {
            emit(std::make_unique<MotionCommand>(
                utility::behaviour::BallApproach(Eigen::Vector2d(3.0, 0.0))));  // TODO: unhack
            currentState = Behaviour::State::WALK_TO_BALL;
        }

        void KickAtGoal::spinToWin() {
            // TODO: does this work?
            Eigen::Affine2d spin = Eigen::Affine2d::Identity();
            spin.linear()        = Eigen::Rotation2Dd(1.0).toRotationMatrix();
            emit(std::make_unique<MotionCommand>(utility::behaviour::DirectCommand(spin)));
            currentState = Behaviour::State::SEARCH_FOR_BALL;
        }
    }  // namespace strategy
}  // namespace behaviour
}  // namespace module
