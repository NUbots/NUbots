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
#include "utility/support/proto_armadillo.h"

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
        handles[Message::IMAGE].push_back(on<Trigger<Image>, Options<Single, Priority<NUClear::LOW>>>([this](const Image& image) {

            if (NUClear::clock::now() - last_image < max_image_duration) {
                return;
            }

            Message message = createMessage(Message::IMAGE, 1);
            
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

        handles[Message::CLASSIFIED_IMAGE].push_back(on<Trigger<ClassifiedImage<ObjectClass>>, Options<Single, Priority<NUClear::LOW>>>([this](const ClassifiedImage<ObjectClass>& image) {

            if (NUClear::clock::now() - last_classified_image < max_classified_image_duration) {
                return;
            }

            Message message = createMessage(Message::CLASSIFIED_IMAGE, 1);
            
            auto* imageData = message.mutable_classified_image();

            imageData->set_camera_id(0);
            *imageData->mutable_dimensions() << image.dimensions;

            // Add the vertical segments to the list
            for(const auto& segment : image.verticalSegments) {
                auto* s = imageData->add_segment();

                s->set_colour(uint(segment.first));
                s->set_subsample(segment.second.subsample);

                *s->mutable_start() << segment.second.start;
                *s->mutable_end() << segment.second.end;
            }

            // Add the horizontal segments to the list
            for(const auto& segment : image.horizontalSegments) {
                auto* s = imageData->add_segment();

                s->set_colour(uint(segment.first));
                s->set_subsample(segment.second.subsample);

                *s->mutable_start() << segment.second.start;
                *s->mutable_end() << segment.second.end;
            }

            // Add in the actual horizon (the points on the left and right side)
            auto* horizon = imageData->mutable_horizon();
            *horizon->mutable_normal() << image.horizon.normal;
            horizon->set_distance(image.horizon.distance);

            for(const auto& visualHorizon : image.visualHorizon) {
                *imageData->add_visual_horizon() << visualHorizon;
            }

            send(message);

            last_classified_image = NUClear::clock::now();
        }));

        handles[Message::VISION_OBJECT].push_back(on<Trigger<std::vector<Ball>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<Ball>& balls) {

            Message message = createMessage(Message::VISION_OBJECT, 1);
            
            auto* object = message.mutable_vision_object();
            object->set_type(VisionObject::BALL);
            object->set_camera_id(0);

            for(const auto& b : balls) {

                auto* ball = object->add_ball();

                auto* circle = ball->mutable_circle();
                circle->set_radius(b.circle.radius);
                *circle->mutable_centre() << b.circle.centre;

                for(auto& measurement : b.measurements) {
                    auto m = ball->add_measurement();
                    *m->mutable_position() << measurement.position;
                    *m->mutable_covariance() << measurement.error;
                }
            }

            send(message);

        }));

        handles[Message::VISION_OBJECT].push_back(on<Trigger<std::vector<Goal>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<Goal>& goals) {

            Message message = createMessage(Message::VISION_OBJECT, 2);
            
            auto* object = message.mutable_vision_object();

            object->set_type(VisionObject::GOAL);
            object->set_camera_id(0);

            for(const auto& g : goals) {
                auto* goal = object->add_goal();

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

            send(message);
        }));

        // TODO: needs refactoring so that this is really only a vision line handle
        handles[Message::VISION_OBJECT].push_back(on<Trigger<VisionObject>, Options<Single, Priority<NUClear::LOW>>>([this] (const VisionObject& visionObject) {

            Message message = createMessage(Message::VISION_OBJECT, 1);
            *message.mutable_vision_object() = visionObject;
            send(message);

        }));

        handles[Message::LOOKUP_TABLE_DIFF].push_back(on<Trigger<LookUpTableDiff>, Options<Single, Priority<NUClear::LOW>>>([this] (const LookUpTableDiff& tableDiff) {
            
            Message message = createMessage(Message::LOOKUP_TABLE_DIFF);
            *message.mutable_lookup_table_diff() = tableDiff;
            send(message);
            
        }));
    }
}
}
