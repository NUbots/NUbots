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

#ifndef MODULES_BEHAVIOUR_SKILLS_LOOKAT_H
#define MODULES_BEHAVIOUR_SKILLS_LOOKAT_H

#include <nuclear>
#include <armadillo>

namespace modules {
    namespace behaviour {
        namespace skills {

            /**
             * Executes a look action.
             *
             * @author Josiah Walker
             */
            class LookAt : public NUClear::Reactor {
            private:
                const size_t id;
                double FAST_SPEED;
                double SLOW_SPEED;
                double HEAD_PITCH_MAX;
                double HEAD_PITCH_MIN;
                double HEAD_YAW_MAX;
                double HEAD_YAW_MIN;
                double SCREEN_EDGE_PADDING;

                std::vector<arma::vec2> currentPoints;
                bool saccading = false;

            public:
                explicit LookAt(std::unique_ptr<NUClear::Environment> environment);
                static constexpr const char* CONFIGURATION_PATH = "LookAt.yaml";
            };

        }  // reflexes
    }  // behaviours
}  // modules

#endif  // MODULES_BEHAVIOURS_REFLEX_LOOK_H

