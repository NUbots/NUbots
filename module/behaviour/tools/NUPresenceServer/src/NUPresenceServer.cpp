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

#include "NUPresenceServer.h"

#include "extension/Configuration.h"

#include "message/input/Image.h"
#include "message/input/ImageFragment.h"
#include "message/input/Sensors.h"

#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace behaviour {
    namespace tools {

        using extension::Configuration;

        using message::input::Image;
        using message::input::Sensors;
        using ServoID = utility::input::ServoID;
        using message::input::ImageFragment;

        using utility::math::matrix::Transform3D;
        using utility::support::Expression;


        NUPresenceServer::NUPresenceServer(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , robot_to_head()
            , robot_to_head_scale(0.0f)
            , reliable(false)
            , camera_to_robot() {

            on<Configuration>("NUPresenceServer.yaml").then([this](const Configuration& config) {
                reliable = config["reliable"];
            });

            on<Configuration>("NUPresenceInput.yaml").then([this](const Configuration& config) {
                // Todo: make this a global config struct message
                float yaw      = config["robot_to_head"]["yaw"].as<Expression>();
                float pitch    = config["robot_to_head"]["pitch"].as<Expression>();
                arma::vec3 pos = config["robot_to_head"]["pos"].as<arma::vec>();

                robot_to_head_scale = config["robot_to_head"]["scale"].as<Expression>();
                robot_to_head       = Transform3D::createTranslation(pos) * Transform3D::createRotationZ(yaw)
                                * Transform3D::createRotationY(pitch);

                arma::vec oculus_x_axis = config["oculus"]["x_axis"].as<arma::vec>();
                arma::vec oculus_y_axis = config["oculus"]["y_axis"].as<arma::vec>();
                arma::vec oculus_z_axis = config["oculus"]["z_axis"].as<arma::vec>();

                camera_to_robot.rotation() =
                    arma::join_rows(oculus_x_axis, arma::join_rows(oculus_y_axis, oculus_z_axis));
            });

            on<Trigger<Image>, With<Sensors>, Single>().then([this](const Image& image, const Sensors& sensors) {
                auto imageFragment = std::make_unique<ImageFragment>();

                imageFragment->image = image;

                imageFragment->start = 0;
                imageFragment->end   = image.data.size();

                Transform3D cam_to_right_foot =
                    convert<double, 4, 4>(sensors.forwardKinematics[ServoID::R_ANKLE_ROLL].inverse()
                                          * sensors.forwardKinematics[ServoID::HEAD_PITCH]);
                Transform3D cam_to_left_foot =
                    convert<double, 4, 4>(sensors.forwardKinematics[ServoID::L_ANKLE_ROLL].inverse()
                                          * sensors.forwardKinematics[ServoID::HEAD_PITCH]);

                Transform3D cam_to_feet   = cam_to_left_foot;
                cam_to_feet.translation() = 0.5 * (cam_to_left_foot.translation() + cam_to_right_foot.translation());
                cam_to_feet               = robot_to_head.i() * cam_to_feet;
                cam_to_feet.translation() /= robot_to_head_scale;

                cam_to_feet = camera_to_robot.t() * cam_to_feet * camera_to_robot;

                // hack out translation
                // TODO: fix translation
                cam_to_feet.translation() *= 0;
                // std::cout << "robot_to_head.i() \n" << robot_to_head.i();
                // std::cout << "cam_to_feet \n" << cam_to_feet;
                imageFragment->cam_to_feet = convert<float, 4, 4>(arma::conv_to<arma::fmat>::from(cam_to_feet));

                emit<Scope::NETWORK>(imageFragment, "nupresenceclient", reliable);
            });

            on<Trigger<NUClear::message::NetworkJoin>>().then(
                [this](const NUClear::message::NetworkJoin& join) { log(join.name); });
        }
    }  // namespace tools
}  // namespace behaviour
}  // namespace module
