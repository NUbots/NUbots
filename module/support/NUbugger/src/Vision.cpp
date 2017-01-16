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

#include "message/input/Image.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/VisionObjects.h"
#include "message/input/proto/Image.h"
#include "message/vision/proto/LookUpTable.h"
#include "message/vision/proto/LookUpTableDiff.h"
#include "message/vision/proto/ClassifiedImage.h"
#include "message/vision/proto/VisionObjects.h"

#include "utility/time/time.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace support {
    using utility::time::getUtcTimestamp;

    using message::input::Sensors;
    using ImageProto = message::input::proto::Image;
    using ClassifiedImageProto = message::vision::proto::ClassifiedImage;
    using message::vision::proto::VisionObjects;
    using message::vision::proto::VisionObject;
    using message::vision::proto::LookUpTableDiff;
    using message::vision::ObjectClass;
    using message::vision::ClassifiedImage;
    using message::vision::Goal;
    using message::vision::Ball;
    using message::input::Image;

    void NUbugger::provideVision() {
        handles["image"].push_back(on<Trigger<Image>, Single, Priority::LOW>().then([this](const Image& image) {

            if (NUClear::clock::now() - last_image < max_image_duration) {
                return;
            }

            ImageProto imageData;

            imageData.camera_id = 0;
            imageData.dimensions.x() = image.width;
            imageData.dimensions.y() = image.height;
            imageData.format = static_cast<uint32_t>(image.fourcc);

            // Reserve enough space in the image data to store the output
            imageData.data.reserve(image.source().size());
            imageData.data.insert(imageData.data.begin(), image.source().begin(), image.source().end());

            send(imageData, 1, false, NUClear::clock::now());

            last_image = NUClear::clock::now();
        }));

        handles["classified_image"].push_back(on<Trigger<ClassifiedImage<ObjectClass>>, Single, Priority::LOW>().then([this](const ClassifiedImage<ObjectClass>& image) {

            if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                return;
            }

            ClassifiedImageProto imageData;

            imageData.camera_id = 0;
            imageData.dimensions = convert<uint, 2>(image.dimensions);

            // Add the vertical segments to the list
            for(const auto& segment : image.verticalSegments) {
                ClassifiedImageProto::Segment protoSegment;
                protoSegment.colour = static_cast<uint32_t>(segment.first);
                protoSegment.subsample = segment.second.subsample;
                protoSegment.start = convert<int, 2>(segment.second.start);
                protoSegment.end = convert<int, 2>(segment.second.end);
                imageData.segment.push_back(protoSegment);
            }

            // Add the horizontal segments to the list
            for(const auto& segment : image.horizontalSegments) {
                ClassifiedImageProto::Segment protoSegment;
                protoSegment.colour = static_cast<uint32_t>(segment.first);
                protoSegment.subsample = segment.second.subsample;
                protoSegment.start = convert<int, 2>(segment.second.start);
                protoSegment.end = convert<int, 2>(segment.second.end);
                imageData.segment.push_back(protoSegment);
            }

            // Add in the actual horizon (the points on the left and right side)
            imageData.horizon.normal   = convert<double, 2>(image.horizon.normal);
            imageData.horizon.distance = image.horizon.distance;

            for(const auto& visualHorizon : image.visualHorizon) {
                imageData.visual_horizon.push_back(convert<int, 2>(visualHorizon));
            }

            send(imageData, imageData.camera_id + 1, false, NUClear::clock::now());

            last_classified_image = NUClear::clock::now();
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Ball>>, Single, Priority::LOW>().then([this] (const std::vector<Ball>& balls) {
            VisionObjects objects;

            VisionObject object;

            object.type = VisionObject::ObjectType::Value::BALL;
            object.camera_id = 0;

            for(const auto& b : balls) {
                VisionObject::Ball ball;
                ball.circle.radius = b.circle.radius;
                ball.circle.centre = convert<double, 2>(b.circle.centre);

                /*
                for (auto& measurement : b.measurements) {
                    VisionObject::Measurement m;
                    m.position   = convert<double, 3>(measurement.position);
                    m.covariance = convert<double, 3, 3>(measurement.error);
                    ball.measurement.push_back(m);
                }
                */

                object.ball.push_back(ball);
            }

            objects.object.push_back(object);
            send(objects, 1, false, NUClear::clock::now());
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Goal>>, Single, Priority::LOW>().then([this] (const std::vector<Goal>& goals) {

            VisionObjects objects;
            VisionObject  object;

            object.type = VisionObject::ObjectType::Value::GOAL;
            object.camera_id = 0;

            for(const auto& g : goals) {
                VisionObject::Goal goal;
                goal.side = (g.side == Goal::Side::LEFT  ? VisionObject::Goal::Side::Value::LEFT
                           : g.side == Goal::Side::RIGHT ? VisionObject::Goal::Side::Value::RIGHT
                           : VisionObject::Goal::Side::Value::UNKNOWN);
                goal.quad.tl = convert<double, 2>(g.quad.getTopLeft());
                goal.quad.tr = convert<double, 2>(g.quad.getTopRight());
                goal.quad.bl = convert<double, 2>(g.quad.getBottomLeft());
                goal.quad.br = convert<double, 2>(g.quad.getBottomRight());

                /*
                for (auto& measurement : g.measurements) {
                    VisionObject::Measurement m;
                    m.position   = convert<double, 3>(measurement.position);
                    m.covariance = convert<double, 3, 3>(measurement.error);
                    goal.measurement.push_back(m);
                }
                */

                object.goal.push_back(goal);
            }

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
