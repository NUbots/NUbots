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

#ifndef MODULE_SUPPORT_VIRTUALGOALPOST
#define MODULE_SUPPORT_VIRTUALGOALPOST

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Goal.h"

namespace module {
namespace support {

    class VirtualGoalPost {
    private:
        Eigen::Vector2d getCamRay(const Eigen::Vector3d& norm1,
                                  const Eigen::Vector3d& norm2,
                                  const message::input::Image::Lens& lens,
                                  const Eigen::Matrix<unsigned int, 2, 1>& dimensions);

    public:
        VirtualGoalPost::VirtualGoalPost(const Eigen::Vector3d& position,
                                         float height,
                                         Goal::Side side,
                                         Goal::Team team)
            : position(position), height(height), side(side), team(team) {}

        Eigen::Vector3d position         = Eigen::Vector3d::Zero();
        float height                     = 1.1;
        message::vision::Goal::Side side = message::vision::Goal::Side::UNKNOWN_SIDE;  // LEFT, RIGHT, or UNKNOWN
        message::vision::Goal::Team team = message::vision::Goal::Team::UNKNOWN_TEAM;  // OWN, OPPONENT, or UNKNOWN

        message::vision::Goals detect(const message::input::Image& image,
                                      const Eigen::Affine2d& robotPose,
                                      const message::input::Sensors& sensors,
                                      const Eigen::Vector4d& /*error*/,
                                      const message::support::FieldDescription& field);
    };
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_VIRTUALGOALPOST
