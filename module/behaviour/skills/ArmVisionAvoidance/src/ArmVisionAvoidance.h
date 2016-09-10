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
 * Copyright 2015 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_SKILLS_ARMVISIONAVOIDANCE_H
#define MODULES_BEHAVIOUR_SKILLS_ARMVISIONAVOIDANCE_H

#include <nuclear>

#include <set>
#include <utility>
#include <vector>

namespace module {
namespace behaviour {
namespace skills {

    class ArmVisionAvoidance : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the ArmController reactor.
        explicit ArmVisionAvoidance(std::unique_ptr<NUClear::Environment> environment);

    private:
        static constexpr size_t UPDATE_FREQUENCY = 90;

        ReactionHandle updateHandle;

        const size_t subsumptionId;

        // The limits on how far the head can be turned before the corresponding arm starts to obscure vision.
        float headYawLimit[2];
        float headPitchLimit;

        // Gain and torque config parameters.
        float gain;
        float torque;
    };

}
}
}

#endif  // MODULES_BEHAVIOUR_SKILLS_ARMVISIONAVOIDANCE_H
