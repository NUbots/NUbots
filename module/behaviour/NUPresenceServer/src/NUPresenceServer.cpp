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

#include "message/support/Configuration.h"

#include "message/input/Image.h"
#include "message/input/proto/ImageFragment.pb.h"

namespace module {
namespace behaviour {

    using message::support::Configuration;

    using message::input::Image;
    using message::input::proto::ImageFragment;

    NUPresenceServer::NUPresenceServer(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NUPresenceServer.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file NUPresenceServer.yaml
        });

        on<Trigger<Image>>().then([this](const Image& image){

        	auto imageFragment = std::make_unique<ImageFragment>();

            imageFragment->set_camera_id(0);
            imageFragment->mutable_dimensions()->set_x(image.width);
            imageFragment->mutable_dimensions()->set_y(image.height);

            imageFragment->set_format(message::input::proto::ImageFragment::YCbCr422);

            // Reserve enough space in the image data to store the output
            std::string* imageBytes = imageFragment->mutable_data();
            imageBytes->reserve(image.source().size());
            imageBytes->insert(imageBytes->begin(), std::begin(image.source()), std::end(image.source()));

            imageFragment->set_start(0);
            imageFragment->set_end(image.source().size());

            emit<Scope::NETWORK>(imageFragment, "nupresenceclient", false);

        });

        on<Trigger<NUClear::message::NetworkJoin>>().then([this](const NUClear::message::NetworkJoin& join){
        	log(join.name);
        });
    }
}
}
