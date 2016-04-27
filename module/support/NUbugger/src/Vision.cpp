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
#include "message/input/proto/Image.pb.h"
#include "message/vision/proto/LookUpTable.pb.h"
#include "message/vision/proto/LookUpTableDiff.pb.h"
#include "message/vision/proto/ClassifiedImage.pb.h"
#include "message/vision/proto/VisionObjects.pb.h"

#include "utility/time/time.h"
#include "utility/support/proto_armadillo.h"

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

            imageData.set_camera_id(0);
            imageData.mutable_dimensions()->set_x(image.width);
            imageData.mutable_dimensions()->set_y(image.height);

            std::string* imageBytes = imageData.mutable_data();
            imageData.set_format(message::input::proto::Image::YCbCr422);

            // Reserve enough space in the image data to store the output
            imageBytes->reserve(image.source().size());
            imageBytes->insert(imageBytes->begin(), std::begin(image.source()), std::end(image.source()));

            send(imageData, 1, false, NUClear::clock::now());

            last_image = NUClear::clock::now();
        }));

        handles["classified_image"].push_back(on<Trigger<ClassifiedImage<ObjectClass>>, Single, Priority::LOW>().then([this](const ClassifiedImage<ObjectClass>& image) {

            if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                return;
            }

            ClassifiedImageProto imageData;

            imageData.set_camera_id(0);
            *imageData.mutable_dimensions() << image.dimensions;

            // Add the vertical segments to the list
            for(const auto& segment : image.verticalSegments) {
                auto* s = imageData.add_segment();

                s->set_colour(uint(segment.first));
                s->set_subsample(segment.second.subsample);

                *s->mutable_start() << segment.second.start;
                *s->mutable_end() << segment.second.end;
            }

            // Add the horizontal segments to the list
            for(const auto& segment : image.horizontalSegments) {
                auto* s = imageData.add_segment();

                s->set_colour(uint(segment.first));
                s->set_subsample(segment.second.subsample);

                *s->mutable_start() << segment.second.start;
                *s->mutable_end() << segment.second.end;
            }

            // Add in the actual horizon (the points on the left and right side)
            auto* horizon = imageData.mutable_horizon();
            *horizon->mutable_normal() << image.horizon.normal;
            horizon->set_distance(image.horizon.distance);

            for(const auto& visualHorizon : image.visualHorizon) {
                *imageData.add_visual_horizon() << visualHorizon;
            }

            send(imageData, imageData.camera_id() + 1, false, NUClear::clock::now());

            last_classified_image = NUClear::clock::now();
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Ball>>, Single, Priority::LOW>().then([this] (const std::vector<Ball>& balls) {

            VisionObjects objects;

            auto& object = *objects.add_object();

            object.set_type(VisionObject::BALL);
            object.set_camera_id(0);

            for(const auto& b : balls) {

                auto* ball = object.add_ball();

                auto* circle = ball->mutable_circle();
                circle->set_radius(b.circle.radius);
                *circle->mutable_centre() << b.circle.centre;

                for(auto& measurement : b.measurements) {
                    auto m = ball->add_measurement();
                    *m->mutable_position() << measurement.position;
                    *m->mutable_covariance() << measurement.error;
                }
            }

            send(object, object.camera_id() + 1, false, NUClear::clock::now());
        }));

        handles["vision_object"].push_back(on<Trigger<std::vector<Goal>>, Single, Priority::LOW>().then([this] (const std::vector<Goal>& goals) {

            VisionObjects objects;

            auto& object = *objects.add_object();

            object.set_type(VisionObject::GOAL);
            object.set_camera_id(0);

            for(const auto& g : goals) {
                auto* goal = object.add_goal();

                goal->set_side(g.side == Goal::Side::LEFT ? VisionObject::Goal::LEFT
                             : g.side == Goal::Side::RIGHT ? VisionObject::Goal::RIGHT
                             : VisionObject::Goal::UNKNOWN);

                auto* quad = goal->mutable_quad();
                *quad->mutable_tl() << g.quad.getTopLeft();
                *quad->mutable_tr() << g.quad.getTopRight();
                *quad->mutable_bl() << g.quad.getBottomLeft();
                *quad->mutable_br() << g.quad.getBottomRight();

                for(auto& measurement : g.measurements) {
                    auto g = goal->add_measurement();
                    *g->mutable_position() << measurement.position;
                    *g->mutable_covariance() << measurement.error;
                }
            }

            send(object, 2, false, NUClear::clock::now());
        }));

        // TODO: needs refactoring so that this is really only a vision line handle
        handles["vision_object"].push_back(on<Trigger<VisionObject>, Single, Priority::LOW>().then([this] (const VisionObject& visionObject) {

            VisionObjects objects;

            *objects.add_object() = visionObject;

            send(objects, 3, false, NUClear::clock::now());
            // NUClear::log("Vision lines emitted");
        }));

        handles["lookup_table_diff"].push_back(on<Trigger<LookUpTableDiff>, Single, Priority::LOW>().then([this] (const LookUpTableDiff& tableDiff) {

            send(tableDiff, 0, true, NUClear::clock::now());
        }));
    }
}
}
