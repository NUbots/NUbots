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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "KickAtGoal.h"

#include "messages/behaviour/Look.h"
#include "messages/localisation/ResetRobotHypotheses.h"
#include "messages/support/FieldDescription.h"
namespace modules {
namespace behaviour {
namespace strategy {
    using messages::support::FieldDescription;
    using messages::behaviour::Look;
    using messages::localisation::ResetRobotHypotheses;

    KickAtGoal::KickAtGoal(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Every<30, Per<std::chrono::seconds>>>, Options<Single>>([this](const time_t&) {

            auto panSelection = std::make_unique<Look::PanSelection>();
            panSelection->lookAtGoalInsteadOfBall = true;
            emit(std::move(panSelection));

        });

        on<Trigger<Startup>>([this](const Startup&){
            FieldDescription desc;

            try {
                desc = *powerplant.get<FieldDescription>();
            }
            catch (NUClear::metaprogramming::NoDataException) {
                throw std::runtime_error("field description get failed asdlfkj");
            }

            auto reset = std::make_unique<ResetRobotHypotheses>();

            ResetRobotHypotheses::Self selfSideBaseLine;
            // selfSideBaseLine.position = arma::vec2({-desc.dimensions.field_length * 0.5 + desc.dimensions.goal_area_length, 0});
            selfSideBaseLine.position = arma::vec2({0, 0});
            selfSideBaseLine.position_cov = arma::eye(2, 2) * 0.1;
            selfSideBaseLine.heading = 0;
            selfSideBaseLine.heading_var = 0.05;
            reset->hypotheses.push_back(selfSideBaseLine);

            emit(std::move(reset));
        });

    }

}
}
}

