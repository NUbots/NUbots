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

#include <zmq.hpp>
#include <jpeglib.h>
#include <cxxabi.h>

#include "messages/input/Sensors.h"
#include "messages/input/Image.h"
#include "messages/vision/ClassifiedImage.h"
#include "messages/vision/VisionObjects.h"
#include "messages/vision/LookUpTable.h"
#include "messages/vision/SaveLookUpTable.h"
#include "messages/localisation/FieldObject.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"

#include "utility/nubugger/NUgraph.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/nubugger/NUgraph.h"
#include "utility/localisation/transform.h"
#include "utility/math/coordinates.h"
// #include "BallModel.h"
#include "utility/image/ColorModelConversions.h"

namespace modules {
    namespace support {


        using utility::nubugger::graph;
        using messages::support::Configuration;
        using messages::input::Sensors;
        using messages::input::Image;
        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        using messages::vision::LookUpTable;
        using std::chrono::duration_cast;
        using std::chrono::microseconds;
        using NUClear::DEBUG;
        using utility::nubugger::graph;
        using messages::support::nubugger::proto::DataPoint;
        using messages::support::nubugger::proto::Message;
        using messages::support::nubugger::proto::Message_Type;
        using messages::vision::Goal;
        using messages::vision::Ball;
        using messages::vision::SaveLookUpTable;

    void NUbugger::EmitLocalisationModels(
    const std::unique_ptr<messages::localisation::FieldObject>& robot_model,
    const std::unique_ptr<messages::localisation::FieldObject>& ball_model) {

        Message message;

        message.set_type(Message::LOCALISATION);
        message.set_utc_timestamp(std::time(0));
        auto* localisation = message.mutable_localisation();

        auto* api_field_object = localisation->add_field_object();
        api_field_object->set_name(robot_model->name);

        for (messages::localisation::FieldObject::Model model : robot_model->models) {
            auto* api_model = api_field_object->add_models();

            api_model->set_wm_x(model.wm_x);
            api_model->set_wm_y(model.wm_y);
            api_model->set_heading(model.heading);
            api_model->set_sd_x(model.sd_x);
            api_model->set_sd_y(model.sd_y);
            api_model->set_sr_xx(model.sr_xx);
            api_model->set_sr_xy(model.sr_xy);
            api_model->set_sr_yy(model.sr_yy);
            api_model->set_lost(model.lost);
        }

        api_field_object = localisation->add_field_object();
        api_field_object->set_name(ball_model->name);

        for (messages::localisation::FieldObject::Model model : ball_model->models) {
            auto* api_model = api_field_object->add_models();

            api_model->set_wm_x(model.wm_x);
            api_model->set_wm_y(model.wm_y);
            api_model->set_heading(model.heading);
            api_model->set_sd_x(model.sd_x);
            api_model->set_sd_y(model.sd_y);
            api_model->set_sr_xx(model.sr_xx);
            api_model->set_sr_xy(model.sr_xy);
            api_model->set_sr_yy(model.sr_yy);
            api_model->set_lost(model.lost);
        }


        send(message);
    }


        NUbugger::NUbugger(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , pub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_PUB)
            , sub(NUClear::extensions::Networking::ZMQ_CONTEXT, ZMQ_SUB) {
            // Set our high water mark
            int hwm = 50;
            pub.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));
            sub.setsockopt(ZMQ_SUBSCRIBE, 0, 0);

            // Bind to port 12000
            pub.bind("tcp://*:12000");
            sub.bind("tcp://*:12001");

            powerplant.addServiceTask(NUClear::threading::ThreadWorker::ServiceTask(std::bind(std::mem_fn(&NUbugger::run), this), std::bind(std::mem_fn(&NUbugger::kill), this)));

            on<Trigger<DataPoint>>([this](const DataPoint& data_point) {
                Message message;
                message.set_type(Message::DATA_POINT);
                message.set_utc_timestamp(std::time(0));

                *message.mutable_datapoint() = data_point;

                send(message);
            });

            // This trigger gets the output from the sensors (unfiltered)
            on<Trigger<Sensors>, Options<Single, Priority<NUClear::LOW>>>([this](const Sensors& sensors) {

                Message message;

                message.set_type(Message::SENSOR_DATA);
                message.set_utc_timestamp(std::time(0));

                auto* sensorData = message.mutable_sensor_data();

                sensorData->set_timestamp(sensors.timestamp.time_since_epoch().count());

                // Add each of the servos into the protocol buffer
                for(const auto& s : sensors.servos) {

                    auto* servo = sensorData->add_servo();

                    servo->set_error_flags(s.errorFlags);

                    servo->set_id(static_cast<messages::input::proto::Sensors_ServoID>(s.id));

                    servo->set_enabled(s.enabled);

                    servo->set_p_gain(s.pGain);
                    servo->set_i_gain(s.iGain);
                    servo->set_d_gain(s.dGain);

                    servo->set_goal_position(s.goalPosition);
                    servo->set_goal_speed(s.goalSpeed);

                    servo->set_present_position(s.presentPosition);
                    servo->set_present_speed(s.presentSpeed);

                    servo->set_load(s.load);
                    servo->set_voltage(s.voltage);
                    servo->set_temperature(s.temperature);
                }

                // The gyroscope values (x,y,z)
                auto* gyro = sensorData->mutable_gyroscope();
                gyro->set_x(sensors.gyroscope[0]);
                gyro->set_y(sensors.gyroscope[1]);
                gyro->set_z(sensors.gyroscope[2]);

                // The accelerometer values (x,y,z)
                auto* accel = sensorData->mutable_accelerometer();
                accel->set_x(sensors.accelerometer[0]);
                accel->set_y(sensors.accelerometer[1]);
                accel->set_z(sensors.accelerometer[2]);

                // The orientation matrix
                auto* orient = sensorData->mutable_orientation();
                orient->set_xx(sensors.orientation(0,0));
                orient->set_yx(sensors.orientation(1,0));
                orient->set_zx(sensors.orientation(2,0));
                orient->set_xy(sensors.orientation(0,1));
                orient->set_yy(sensors.orientation(1,1));
                orient->set_zy(sensors.orientation(2,1));
                orient->set_xz(sensors.orientation(0,2));
                orient->set_yz(sensors.orientation(1,2));
                orient->set_zz(sensors.orientation(2,2));

                // The left FSR values
                auto* lfsr = sensorData->mutable_left_fsr();
                lfsr->set_x(sensors.leftFSR[0]);
                lfsr->set_y(sensors.leftFSR[1]);
                lfsr->set_z(sensors.leftFSR[2]);

                // The right FSR values
                auto* rfsr = sensorData->mutable_right_fsr();
                rfsr->set_x(sensors.rightFSR[0]);
                rfsr->set_y(sensors.rightFSR[1]);
                rfsr->set_z(sensors.rightFSR[2]);

                send(message);
            });

            on<Trigger<Image>, Options<Single, Priority<NUClear::LOW>>>([this](const Image& image) {

                if(!image.source().empty()) {

                    Message message;
                    message.set_type(Message::IMAGE);
                    message.set_utc_timestamp(std::time(0));

                    auto* imageData = message.mutable_image();
                    std::string* imageBytes = imageData->mutable_data();

                    // Reserve enough space in the image data to store the output
                    imageBytes->resize(image.source().size());
                    imageData->set_width(image.width());
                    imageData->set_height(image.height());

                    imageBytes->insert(imageBytes->begin(), std::begin(image.source()), std::end(image.source()));

                    send(message);
                }
            });

            // on<Trigger<NUClear::ReactionStatistics>>([this](const NUClear::ReactionStatistics& stats) {
            //     Message message;
            //     message.set_type(Message::REACTION_STATISTICS);
            //     message.set_utc_timestamp(std::time(0));

            //     auto* reactionStatistics = message.mutable_reactionstatistics();

            //     //reactionStatistics->set_name(stats.name);
            //     reactionStatistics->set_reactionid(stats.reactionId);
            //     reactionStatistics->set_taskid(stats.taskId);
            //     reactionStatistics->set_causereactionid(stats.causeReactionId);
            //     reactionStatistics->set_causetaskid(stats.causeTaskId);
            //     reactionStatistics->set_emitted(duration_cast<microseconds>(stats.emitted.time_since_epoch()).count());
            //     reactionStatistics->set_started(duration_cast<microseconds>(stats.started.time_since_epoch()).count());
            //     reactionStatistics->set_finished(duration_cast<microseconds>(stats.finished.time_since_epoch()).count());

            //     reactionStatistics->set_name(stats.identifier[0]);
            //     reactionStatistics->set_triggername(stats.identifier[1]);
            //     reactionStatistics->set_functionname(stats.identifier[2]);

            //     send(message);
            // });

            on<Trigger<ClassifiedImage<ObjectClass>>, Options<Single, Priority<NUClear::LOW>>>([this](const ClassifiedImage<ObjectClass>& image) {

                Message message;
                message.set_type(Message::VISION);
                message.set_utc_timestamp(std::time(0));

                auto* imageData = message.mutable_classified_image();

                // Add the vertical segments to the list
                for(const auto& segment : image.verticalSegments) {
                    auto* s = imageData->add_segment();

                    s->set_colour(uint(segment.first));

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

                    auto* start = s->mutable_start();
                    start->set_x(segment.second.start[0]);
                    start->set_y(segment.second.start[1]);

                    auto* end = s->mutable_end();
                    end->set_x(segment.second.end[0]);
                    end->set_y(segment.second.end[1]);
                }

                // Add in the actual horizon (the points on the left and right side)
                auto* horizon = imageData->mutable_horizon();
                horizon->set_x(image.horizon[0]);
                horizon->set_y(image.horizon[0]);

                for(const auto& visualHorizon : image.visualHorizon) {
                    auto* vh = imageData->add_visual_horizon();

                    vh->set_x(visualHorizon[0]);
                    vh->set_y(visualHorizon[1]);
                }

                send(message);
            });

            /*on<Trigger<ClassifiedImage>, Options<Single, Priority<NUClear::LOW>>>([this](const ClassifiedImage& image) {

                Message message;
                message.set_type(Message::VISION);
                message.set_utc_timestamp(std::time(0));
                Message::Vision* api_vision = message.mutable_vision();

                Message::VisionClassifiedImage* api_classified_image = api_vision->mutable_classified_image();

                for (auto& rowColourSegments : image.horizontalFilteredSegments.m_segmentedScans) {
                    for (auto& colorSegment : rowColourSegments) {
                        auto& start = colorSegment.m_start;
                        auto& end = colorSegment.m_end;
                        auto& colour = colorSegment.m_colour;

                        Message::VisionClassifiedSegment* api_segment = api_classified_image->add_segment();
                        api_segment->set_start_x(start[0]);
                        api_segment->set_start_y(start[1]);
                        api_segment->set_end_x(end[0]);
                        api_segment->set_end_y(end[1]);
                        api_segment->set_colour(colour);
                    }
                }

                for (auto& columnColourSegments : image.verticalFilteredSegments.m_segmentedScans)
                {
                    for (auto& colorSegment : columnColourSegments)
                    {
                        auto& start = colorSegment.m_start;
                        auto& end = colorSegment.m_end;
                        auto& colour = colorSegment.m_colour;

                        Message::VisionClassifiedSegment* api_segment = api_classified_image->add_segment();
                        api_segment->set_start_x(start[0]);
                        api_segment->set_start_y(start[1]);
                        api_segment->set_end_x(end[0]);
                        api_segment->set_end_y(end[1]);
                        api_segment->set_colour(colour);
                    }
                }

                for (auto& matchedSegment : image.matchedVerticalSegments)
                {
                    for (auto& columnColourSegment : matchedSegment.second)
                    {
                        auto& start = columnColourSegment.m_start;
                        auto& end = columnColourSegment.m_end;
                        auto& colour = columnColourSegment.m_colour;
                        auto& colourClass = matchedSegment.first;

                        Message::VisionTransitionSegment* api_segment = api_classified_image->add_transition_segment();
                        api_segment->set_start_x(start[0]);
                        api_segment->set_start_y(start[1]);
                        api_segment->set_end_x(end[0]);
                        api_segment->set_end_y(end[1]);
                        api_segment->set_colour(colour);
                        api_segment->set_colour_class(colourClass);
                    }
                }

                for (auto& matchedSegment : image.matchedHorizontalSegments)
                {
                    for (auto& rowColourSegment : matchedSegment.second)
                    {
                        auto& start = rowColourSegment.m_start;
                        auto& end = rowColourSegment.m_end;
                        auto& colour = rowColourSegment.m_colour;
                        auto& colourClass = matchedSegment.first;

                        Message::VisionTransitionSegment* api_segment = api_classified_image->add_transition_segment();
                        api_segment->set_start_x(start[0]);
                        api_segment->set_start_y(start[1]);
                        api_segment->set_end_x(end[0]);
                        api_segment->set_end_y(end[1]);
                        api_segment->set_colour(colour);
                        api_segment->set_colour_class(colourClass);
                    }
                }

                for (auto& greenHorizonPoint : image.greenHorizonInterpolatedPoints) {
                    Message::VisionGreenHorizonPoint* api_ghpoint = api_classified_image->add_green_horizon_point();
                    api_ghpoint->set_x(greenHorizonPoint[0]);
                    api_ghpoint->set_y(greenHorizonPoint[1]);
                }

                send(message);

            });*/
            /*
            on<Trigger<std::vector<Goal>>, Options<Single, Priority<NUClear::LOW>>>([this](const std::vector<Goal> goals){
                Message message;

                message.set_type(Message::VISION);
                message.set_utc_timestamp(std::time(0));

                Message::Vision* api_vision = message.mutable_vision();
                //NUClear::log("NUbugger emmitting goals: ", goals.size());
                for (auto& goal : goals){
                    Message::VisionFieldObject* api_goal = api_vision->add_vision_object();

                    api_goal->set_shape_type(Message::VisionFieldObject::QUAD);
                    api_goal->set_goal_type(Message::VisionFieldObject::GoalType(1+int(goal.type))); //+1 to account for zero vs one referencing in message buffer.
                    api_goal->set_name("Goal");
                    api_goal->set_width(goal.sizeOnScreen[0]);
                    api_goal->set_height(goal.sizeOnScreen[1]);
                    api_goal->set_screen_x(goal.screenCartesian[0]);
                    api_goal->set_screen_y(goal.screenCartesian[1]);

                    arma::vec3 goal_pos = utility::math::coordinates::Spherical2Cartesian(goal.sphericalFromNeck);

                    switch(int(goal.type)){
                        case(0):
                            emit(graph("Left vgoal", goal_pos[0], goal_pos[1], goal_pos[2]));
                            break;
                        case(1):
                            emit(graph("Right vgoal", goal_pos[0], goal_pos[1], goal_pos[2]));
                            break;
                        case(2):
                            emit(graph("Ambiguous vgoal", goal_pos[0], goal_pos[1], goal_pos[2]));
                    }


                    for(auto& point : goal.screen_quad){
                        api_goal->add_points(point[0]);
                        api_goal->add_points(point[1]);
                        //std::cout<< "NUbugger::on<Trigger<std::vector<Goal>>> : adding quad point ( " << point[0] << " , " << point[1] << " )."<< std::endl;
                    }
                    for(auto& coord : goal.sphericalFromNeck){
                        api_goal->add_measured_relative_position(coord);
                    }
                }
                send(message);
            });

            on<Trigger<std::vector<Ball>>, Options<Single, Priority<NUClear::LOW>>>([this](const std::vector<Ball> balls){
                Message message;

                message.set_type(Message::VISION);
                message.set_utc_timestamp(std::time(0));

                Message::Vision* api_vision = message.mutable_vision();
                //std::cout<< "NUbugger::on<Trigger<std::vector<Ball>>> : sending " << balls.size() << " balls to NUbugger." << std::endl;
                if(balls.size()>0){
                    arma::vec3 first_ball_pos= utility::math::coordinates::Spherical2Cartesian(balls[0].sphericalFromNeck);
                    emit(graph("Vision Ball pos", first_ball_pos[0], first_ball_pos[1],first_ball_pos[2]));
                }
                for (auto& ball : balls){
                    Message::VisionFieldObject* api_ball = api_vision->add_vision_object();

                    api_ball->set_shape_type(Message::VisionFieldObject::CIRCLE);
                    api_ball->set_name("Ball");
                    api_ball->set_width(ball.sizeOnScreen[0]);
                    api_ball->set_height(ball.sizeOnScreen[1]);
                    api_ball->set_screen_x(ball.screenCartesian[0]);
                    api_ball->set_screen_y(ball.screenCartesian[1]);

                    api_ball->set_radius(ball.diameter / 2.0);
                    for(auto& coord : ball.sphericalFromNeck){
                        api_ball->add_measured_relative_position(coord);
                    }
                }
                send(message);
            });*/

            on<Trigger<Every<100, std::chrono::milliseconds>>,
               With<messages::localisation::Ball>,
               // With<messages::vision::Ball>,
               With<std::vector<messages::localisation::Self>>,
               Options<Single>>("Localisation Ball",
                [this](const time_t&,
                       const messages::localisation::Ball& ball,
                       // const messages::vision::Ball& vision_ball,
                       const std::vector<messages::localisation::Self>& robots) {
                if(robots.size() > 0){
                    arma::vec2 ball_pos = utility::localisation::transform::RobotBall2FieldBall(
                        robots[0].position,robots[0].heading, ball.position);

                    // Ball message
                    auto ball_msg = std::make_unique<messages::localisation::FieldObject>();
                    std::vector<messages::localisation::FieldObject::Model> ball_msg_models;
                    messages::localisation::FieldObject::Model ball_model;
                    ball_msg->name = "ball";
                    ball_model.wm_x = ball_pos[0];
                    ball_model.wm_y = ball_pos[1];
                    ball_model.heading = 0;
                    ball_model.sd_x = 0.1;
                    ball_model.sd_y = 0.1;

                    //Do we need to rotate the variances?
                    ball_model.sr_xx = ball.sr_xx;
                    ball_model.sr_xy = ball.sr_xy;
                    ball_model.sr_yy = ball.sr_yy;
                    ball_model.lost = false;
                    ball_msg_models.push_back(ball_model);

                    ball_msg->models = ball_msg_models;
                    // emit(std::move(ball_msg));

                    // Robot message
                    auto robot_msg = std::make_unique<messages::localisation::FieldObject>();
                    std::vector<messages::localisation::FieldObject::Model> robot_msg_models;

                    //for (auto& model : robots) {
                    auto model = robots[0];
                        messages::localisation::FieldObject::Model robot_model;
                        robot_msg->name = "self";
                        robot_model.wm_x = model.position[0];
                        robot_model.wm_y = model.position[1];
                        robot_model.heading = std::atan2(model.heading[1], model.heading[0]);
                        robot_model.sd_x = 1;
                        robot_model.sd_y = 0.25;
                        robot_model.sr_xx = model.sr_xx; // * 100;
                        robot_model.sr_xy = model.sr_xy; // * 100;
                        robot_model.sr_yy = model.sr_yy; // * 100;
                        robot_model.lost = false;
                        robot_msg_models.push_back(robot_model);
                    //}


                    robot_msg->models = robot_msg_models;
                    // emit(std::move(robot_msg));

                    EmitLocalisationModels(robot_msg, ball_msg);
                }
            });

            // When we shutdown, close our publisher
            on<Trigger<Shutdown>>([this](const Shutdown&) {
                pub.close();
            });
        }

        void NUbugger::run() {
            // TODO: fix this - still blocks on last recv even if listening = false
            while (listening) {
                zmq::message_t message;
                sub.recv(&message);

                // If our message size is 0, then it is probably our termination message
                if (message.size() > 0) {

                    // Parse our message
                    Message proto;
                    proto.ParseFromArray(message.data(), message.size());
                    recvMessage(proto);
                }
            }
        }

        void NUbugger::recvMessage(const Message& message) {
            NUClear::log("Received message of type:", message.type());
            switch (message.type()) {
                case Message::Type::Message_Type_COMMAND:
                    recvCommand(message);
                    break;
                case Message::Type::Message_Type_LOOKUP_TABLE:
                    recvLookupTable(message);
                    break;
                default:
                    return;
            }
        }

        void NUbugger::recvCommand(const Message& message) {
            std::string command = message.command().command();
            NUClear::log("Received command:", command);
            if (command == "download_lut") {
                auto lut = powerplant.get<LookUpTable>();

                Message message;

                message.set_type(Message::LOOKUP_TABLE);
                message.set_utc_timestamp(std::time(0));

                auto* api_lookup_table = message.mutable_lookup_table();
                api_lookup_table->set_table(lut->getData());

                send(message);
            }
        }

        void NUbugger::recvLookupTable(const Message& message) {
            auto lookuptable = message.lookup_table();
            const std::string& lutData = lookuptable.table();

            NUClear::log("Loading LUT");
            auto data = std::unique_ptr<char[]>(new char[lutData.size()]);
            std::copy(std::begin(lutData), std::end(lutData), data.get());
            auto lut = std::make_unique<LookUpTable>(lookuptable.bits_y(), lookuptable.bits_cb(), lookuptable.bits_cr(), std::move(data));
            emit<Scope::DIRECT>(std::move(lut));

            if (lookuptable.save()) {
                NUClear::log("Saving LUT to file");
                emit<Scope::DIRECT>(std::make_unique<SaveLookUpTable>());
            }
        }

        void NUbugger::kill() {
            listening = false;
        }

        /**
         * This method needs to be used over pub.send as all calls to
         * pub.send need to be synchronized with a concurrency primitive
         * (such as a mutex)
         */
        void NUbugger::send(zmq::message_t& packet) {
            std::lock_guard<std::mutex> lock(mutex);
            pub.send(packet);
        }

        void NUbugger::send(Message message) {
            auto serialized = message.SerializeAsString();
            zmq::message_t packet(serialized.size());
            memcpy(packet.data(), serialized.data(), serialized.size());
            send(packet);
        }

    } // support
} // modules
