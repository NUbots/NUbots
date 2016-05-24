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
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_BEHAVIOUR_NUPRESENCESERVER_H
#define MODULE_BEHAVIOUR_NUPRESENCESERVER_H

#include <nuclear>
#include "utility/math/matrix/Transform3D.h"

namespace module {
namespace behaviour {

    class NUPresenceServer : public NUClear::Reactor {
    private:
    	utility::math::matrix::Transform3D robot_to_head;
        float robot_to_head_scale;
        
        bool reliable;
        utility::math::matrix::Transform3D camera_to_robot;
    public:
        /// @brief Called by the powerplant to build and setup the NUPresenceServer reactor.
        explicit NUPresenceServer(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_BEHAVIOUR_NUPRESENCESERVER_H
