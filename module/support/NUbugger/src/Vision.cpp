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
    using message::vision::proto::VisionObjects;
    using message::vision::proto::VisionObject;
    using message::vision::proto::LookUpTableDiff;
    using message::vision::proto::ClassifiedImage;

    void NUbugger::provideVision() {
        handles["image"].push_back(on<Trigger<Image>, Single, Priority::LOW>().then([this](const Image& image) {

            if (NUClear::clock::now() - last_image < max_image_duration) {
                return;
            }

            send(image, 1, false, NUClear::clock::now());

            last_image = NUClear::clock::now();
        }));

        handles["classified_image"].push_back(on<Trigger<ClassifiedImage>, Single, Priority::LOW>().then([this](const ClassifiedImage& image) {

            if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                return;
            }

            send(image, image.image->camera_id + 1, false, NUClear::clock::now());

            last_classified_image = NUClear::clock::now();
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<VisionObject::Ball>>, Single, Priority::LOW>().then([this] (const std::vector<VisionObject::Ball>& balls) {
            VisionObjects objects;

            VisionObject object;

            object.type = VisionObject::ObjectType::Value::BALL;
            object.camera_id = 0;
            object.ball = balls;
            objects.object.push_back(object);

            send(objects, 1, false, NUClear::clock::now());
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<VisionObject::Goal>>, Single, Priority::LOW>().then([this] (const std::vector<VisionObject::Goal>& goals) {

            VisionObjects objects;
            VisionObject  object;

            object.type = VisionObject::ObjectType::Value::GOAL;
            object.camera_id = 0;
            object.goal = goals;
            objects.object.push_back(object);
            
            send(objects, 2, false, NUClear::clock::now());
        }));

        // TODO: needs refactoring so that this is really only a vision line handle
        handles["vision_object"].push_back(on<Trigger<VisionObject>, Single, Priority::LOW>().then([this] (const VisionObject& visionObject) {

            VisionObjects objects;

            objects.object.push_back(visionObject);

            send(objects, 3, false, NUClear::clock::now());
            // NUClear::log("Vision lines emitted");
        }));

        handles["lookup_table_diff"].push_back(on<Trigger<LookUpTableDiff>, Single, Priority::LOW>().then([this] (const LookUpTableDiff& tableDiff) {

            send(tableDiff, 0, true, NUClear::clock::now());
        }));
    }
}
}
