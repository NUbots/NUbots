/*
 * This file is part of PS3Walk.
 *
 * PS3Walk is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PS3Walk is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with PS3Walk.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_BEHAVIOUR_PLANNING_PS3WALK_H
#define MODULES_BEHAVIOUR_PLANNING_PS3WALK_H


#include <nuclear>
#include <armadillo>
#include "Joystick.h"

namespace modules {
namespace behaviour {
namespace planning {

    class PS3Walk : public NUClear::Reactor {
    public:
        static constexpr uint LEFT_JOYSTICK_HORIZONTAL = 0;
        static constexpr uint LEFT_JOYSTICK_VERTICAL = 1;
        static constexpr uint RIGHT_JOYSTICK_HORIZONTAL = 2;
        static constexpr uint RIGHT_JOYSTICK_VERTICAL = 3;

        /// @brief Called by the powerplant to build and setup the PS3Walk reactor.
        explicit PS3Walk(std::unique_ptr<NUClear::Environment> environment);
    private:
        Joystick joystick{"/dev/input/js1"};
        arma::vec2 strafe{0, 0};
        float rotationalSpeed = 0;
    };

}
}
}


#endif