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

#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/vision/proto/LookUpTable.pb.h"
#include "messages/input/Image.h"
#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/VisionObjects.h"

#include "utility/time/time.h"

namespace modules {
namespace support {
    using messages::support::nubugger::proto::Message;
    using utility::time::getUtcTimestamp;

    using messages::input::Sensors;
    using messages::vision::proto::VisionObject;
    using messages::vision::proto::LookUpTableDiff;
    using messages::vision::ObjectClass;
    using messages::vision::ClassifiedImage;
    using messages::vision::Goal;
    using messages::vision::Ball;
    using messages::input::Image;

    void NUbugger::provideVision() {
        handles["image"].push_back(on<Trigger<Image>, Options<Single, Priority<NUClear::LOW>>>([this](const Image& image) {

            if (NUClear::clock::now() - last_image < max_image_duration) {
                return;
            }

            Message message;
            message.set_type(Message::IMAGE);
            message.set_filter_id(1);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* imageData = message.mutable_image();

            imageData->set_camera_id(0);
            imageData->mutable_dimensions()->set_x(image.width);
            imageData->mutable_dimensions()->set_y(image.height);

            std::string* imageBytes = imageData->mutable_data();
            imageData->set_format(messages::input::proto::Image::YCbCr422);

            // Reserve enough space in the image data to store the output
            imageBytes->reserve(image.source().size());
            imageBytes->insert(imageBytes->begin(), std::begin(image.source()), std::end(image.source()));

            send(message);

            last_image = NUClear::clock::now();
        }));

        handles["classified_image"].push_back(on<Trigger<ClassifiedImage<ObjectClass>>, Options<Single, Priority<NUClear::LOW>>>([this](const ClassifiedImage<ObjectClass>& image) {

            if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                return;
            }

            Message message;
            message.set_type(Message::CLASSIFIED_IMAGE);
            message.set_filter_id(1);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* imageData = message.mutable_classified_image();

            imageData->set_camera_id(0);
            imageData->mutable_dimensions()->set_x(image.dimensions[0]);
            imageData->mutable_dimensions()->set_y(image.dimensions[1]);

            // Add the vertical segments to the list
            for(const auto& segment : image.verticalSegments) {
                auto* s = imageData->add_segment();

                s->set_colour(uint(segment.first));
                s->set_subsample(segment.second.subsample);

                auto* start = s->mutable_start();
                start->set_x(segment.second.start[0]);
                start->set_y(segment.second.start[1]);

                auto* end = s->mutable_end();
                end->set_x(segment.second.end[0]);
                end->set_y(segment.second.end[1]);
            }

            // Add the horizontal segments to the list
            for(const auto& segment : image.horizontalSegments) {
                auto* s = imageData->add_segment();

                s->set_colour(uint(segment.first));
                s->set_subsample(segment.second.subsample);

                auto* start = s->mutable_start();
                start->set_x(segment.second.start[0]);
                start->set_y(segment.second.start[1]);

                auto* end = s->mutable_end();
                end->set_x(segment.second.end[0]);
                end->set_y(segment.second.end[1]);
            }

            // Add in the actual horizon (the points on the left and right side)
            auto* horizon = imageData->mutable_horizon();
            horizon->mutable_normal()->set_x(image.horizon.normal[0]);
            horizon->mutable_normal()->set_y(image.horizon.normal[1]);
            horizon->set_distance(image.horizon.distance);

            for(const auto& visualHorizon : image.visualHorizon) {
                auto* vh = imageData->add_visual_horizon();

                vh->set_x(visualHorizon[0]);
                vh->set_y(visualHorizon[1]);
            }

            send(message);

            last_classified_image = NUClear::clock::now();
        }));

        handles["balls"].push_back(on<Trigger<std::vector<Ball>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<Ball>& balls) {

            Message message;
            message.set_type(Message::VISION_OBJECT);
            message.set_filter_id(1);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* object = message.mutable_vision_object();
            object->set_type(VisionObject::BALL);
            object->set_camera_id(0);

            for(const auto& b : balls) {

                auto* ball = object->add_ball();

                auto* circle = ball->mutable_circle();
                circle->set_radius(b.circle.radius);
                circle->mutable_centre()->set_x(b.circle.centre[0]);
                circle->mutable_centre()->set_y(b.circle.centre[1]);
            }

            send(message);

        }));

        handles["goals"].push_back(on<Trigger<std::vector<Goal>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<Goal>& goals) {

            Message message;
            message.set_type(Message::VISION_OBJECT);
            message.set_filter_id(2);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* object = message.mutable_vision_object();

            object->set_type(VisionObject::GOAL);
            object->set_camera_id(0);

            for(const auto& g : goals) {
                auto* goal = object->add_goal();

                goal->set_side(g.side == Goal::Side::LEFT ? VisionObject::Goal::LEFT
                             : g.side == Goal::Side::RIGHT ? VisionObject::Goal::RIGHT
                             : VisionObject::Goal::UNKNOWN);

                auto* quad = goal->mutable_quad();
                quad->mutable_tl()->set_x(g.quad.getTopLeft()[0]);
                quad->mutable_tl()->set_y(g.quad.getTopLeft()[1]);
                quad->mutable_tr()->set_x(g.quad.getTopRight()[0]);
                quad->mutable_tr()->set_y(g.quad.getTopRight()[1]);
                quad->mutable_bl()->set_x(g.quad.getBottomLeft()[0]);
                quad->mutable_bl()->set_y(g.quad.getBottomLeft()[1]);
                quad->mutable_br()->set_x(g.quad.getBottomRight()[0]);
                quad->mutable_br()->set_y(g.quad.getBottomRight()[1]);
            }

            send(message);
        }));

        // TODO: needs refactoring so that this is really only a vision line handle
        handles["vision_lines"].push_back(on<Trigger<VisionObject>, Options<Single, Priority<NUClear::LOW>>>([this] (const VisionObject& visionObject) {

            Message message;
            message.set_type(Message::VISION_OBJECT);
            message.set_filter_id(1);
            message.set_utc_timestamp(getUtcTimestamp());

            *message.mutable_vision_object() = visionObject;

            send(message);

        }));

        handles["lookuptable"].push_back(on<Trigger<LookUpTableDiff>, Options<Single, Priority<NUClear::LOW>>>([this] (const LookUpTableDiff& tableDiff) {
            Message message;
            message.set_type(Message::LOOKUP_TABLE_DIFF);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());
            *message.mutable_lookup_table_diff() = tableDiff;

            send(message);
        }));
    }
}
}
