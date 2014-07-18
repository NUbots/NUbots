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

#include "LookPlanner.h"

#include "messages/behaviour/LookStrategy.h"
#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"

namespace modules {
namespace behaviour {
namespace planning {

    using messages::behaviour::LookStrategy;

    using VisionBall = messages::vision::Ball;
    using VisionGoal = messages::vision::Goal;

    using LocalisationBall = messages::localisation::Ball;
    using messages::localisation::Self;

    LookPlanner::LookPlanner(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Last<5, std::vector<VisionBall>>>, With<Optional<LocalisationBall>>, With<LookStrategy>>([this] (const LastList<std::vector<VisionBall>>& v, const std::shared_ptr<const LocalisationBall>& l, const LookStrategy& strat) {
            //

            const auto& vision = **std::find_if(v.rbegin(), v.rend(), [] (const std::shared_ptr<const std::vector<VisionBall>>& a) {
                return a->empty();
            });

            (void)v;
            (void)l;
            (void)strat;
        });

        on<Trigger<Last<5, std::vector<VisionGoal>>>, With<Optional<Self>>, With<LookStrategy>>([this] (const LastList<std::vector<VisionGoal>>& v, const std::shared_ptr<const Self>& self, const LookStrategy& strat) {
            (void)v;
            (void)self;
            (void)strat;
        });
    }

}
}
}
