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

#include "NUcapSimulator.h"

#include "message/support/Configuration.h"
#include "message/input/proto/MotionCapture.pb.h"

namespace module {
namespace support {

    using message::support::Configuration;
    using message::input::proto::MotionCapture;

    NUcapSimulator::NUcapSimulator(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NUcapSimulator.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file NUcapSimulator.yaml
            auto moCap = std::make_unique<MotionCapture>();

            auto* rigidBody = moCap->add_rigid_bodies();
            rigidBody->set_identifier(config["id"].as<int>());

            auto* position = rigidBody->mutable_position();
            // normalize to robot coordinate system, x foward, y left, z up
            position->set_x(config["position_x"].as<float>());
            position->set_y(config["position_y"].as<float>());
            position->set_z(config["position_z"].as<float>());

            auto* rotation = rigidBody->mutable_rotation();
            rotation->set_x(config["rotation_x"].as<float>());
            rotation->set_y(config["rotation_y"].as<float>());
            rotation->set_z(config["rotation_z"].as<float>());
            rotation->set_t(config["rotation_t"].as<float>());

            emit<Scope::NETWORK,Scope::LOCAL>(std::move(moCap));
        });

    }
}
}
