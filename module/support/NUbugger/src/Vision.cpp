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

#include "NUbugger.h"

#include "message/input/proto/Image.h"
#include "message/vision/proto/ClassifiedImage.h"
#include "message/vision/proto/VisionObjects.h"
#include "message/vision/proto/LookUpTable.h"
#include "message/vision/proto/LookUpTableDiff.h"
#include "message/vision/proto/ClassifiedImage.h"

#include "utility/time/time.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using message::input::proto::Image;
    using message::vision::proto::Ball;
    using message::vision::proto::Goal;
    using message::vision::proto::Line;
    using message::vision::proto::Obstacle;
    using message::vision::proto::LookUpTableDiff;
    using message::vision::proto::ClassifiedImage;

    void NUbugger::provideVision() {
        handles["image"].push_back(on<Trigger<Image>, Single, Priority::LOW>().then([this](const Image& image) {

            if (NUClear::clock::now() - last_image < max_image_duration) {
                return;
            }

            auto imageData = image;
            send<Image>(imageData, 1, false, NUClear::clock::now());
            //send<Image>(std::make_unique<Image>(image), 1, false, NUClear::clock::now());

            last_image = NUClear::clock::now();
        }));

        handles["classified_image"].push_back(on<Trigger<ClassifiedImage>, Single, Priority::LOW>().then([this](const ClassifiedImage& image) {

            if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                return;
            }

            send<ClassifiedImage>(std::make_unique<ClassifiedImage>(image), image.image->camera_id + 1, false, NUClear::clock::now());

            last_classified_image = NUClear::clock::now();
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Ball>>, Single, Priority::LOW>().then([this] (const std::vector<Ball>& balls) {

            send<Ball>(std::make_unique<std::vector<Ball>>(balls), 1, false, NUClear::clock::now());
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Goal>>, Single, Priority::LOW>().then([this] (const std::vector<Goal>& goals) {

            send<Goal>(std::make_unique<std::vector<Goal>>(goals), 2, false, NUClear::clock::now());
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Line>>, Single, Priority::LOW>().then([this] (const std::vector<Line>& lines) {

            send<Line>(std::make_unique<std::vector<Line>>(lines), 3, false, NUClear::clock::now());
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Obstacle>>, Single, Priority::LOW>().then([this] (const std::vector<Obstacle>& obstacles) {

            send<Obstacle>(std::make_unique<std::vector<Obstacle>>(obstacles), 4, false, NUClear::clock::now());
        }));

        handles["lookup_table_diff"].push_back(on<Trigger<LookUpTableDiff>, Single, Priority::LOW>().then([this] (const LookUpTableDiff& tableDiff) {

            send<LookUpTableDiff>(std::make_unique<LookUpTableDiff>(tableDiff), 0, true, NUClear::clock::now());
        }));
    }
}
}
