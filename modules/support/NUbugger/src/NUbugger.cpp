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
#include "messages/behaviour/Action.h"
#include "messages/behaviour/proto/Behaviour.pb.h"

#include "utility/nubugger/NUgraph.h"
#include "utility/time/time.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/nubugger/NUgraph.h"
#include "utility/localisation/transform.h"
#include "utility/math/coordinates.h"
// #include "BallModel.h"
#include "utility/image/ColorModelConversions.h"

namespace modules {
    namespace support {

        using std::chrono::duration_cast;
        using std::chrono::microseconds;

        using utility::nubugger::graph;
        using utility::time::getUtcTimestamp;

        using messages::support::Configuration;
        using messages::support::nubugger::proto::DataPoint;
        using messages::support::nubugger::proto::Message;

        using messages::input::Sensors;
        using messages::input::Image;

        using messages::behaviour::ActionStart;
        using messages::behaviour::ActionKill;
        using messages::behaviour::RegisterAction;
        using messages::behaviour::proto::Behaviour;
        using messages::behaviour::proto::ActionStateChange;

        using messages::vision::ObjectClass;
        using messages::vision::ClassifiedImage;
        using messages::vision::LookUpTable;
        using VisionGoal = messages::vision::Goal;
        using VisionBall = messages::vision::Ball;
        using messages::vision::SaveLookUpTable;

        using messages::localisation::FieldObject;

        void NUbugger::EmitLocalisationModels(const std::unique_ptr<FieldObject>& robot_model, const std::unique_ptr<FieldObject>& ball_model) {

            Message message;

            message.set_type(Message::LOCALISATION);
            message.set_utc_timestamp(getUtcTimestamp());
            auto* localisation = message.mutable_localisation();

            auto* api_field_object = localisation->add_field_object();
            api_field_object->set_name(robot_model->name);

            for (FieldObject::Model model : robot_model->models) {
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

            for (FieldObject::Model model : ball_model->models) {
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

            dataPointHandle = on<Trigger<DataPoint>>([this](const DataPoint& data_point) {
                Message message;
                message.set_type(Message::DATA_POINT);
                message.set_utc_timestamp(getUtcTimestamp());

            *(message.mutable_data_point()) = data_point;

                send(message);
            });

            actionStartHandle = on<Trigger<ActionStart>>([this](const ActionStart& actionStart) {
               Message message;
               message.set_type(Message::BEHAVIOUR);
               message.set_utc_timestamp(getUtcTimestamp());

               auto* behaviour = message.mutable_behaviour();
               behaviour->set_type(Behaviour::ACTION_STATE);
               auto* actionStateChange = behaviour->mutable_action_state_change();
               actionStateChange->set_state(ActionStateChange::START);
               actionStateChange->set_name(actionStart.name);
               for (auto& limbID : actionStart.limbs) {
                   actionStateChange->add_limbs(static_cast<int>(limbID));
               }

                send(message);
            });

            actionKillHandle = on<Trigger<ActionKill>>([this](const ActionKill& actionKill) {
               Message message;
               message.set_type(Message::BEHAVIOUR);
               message.set_utc_timestamp(getUtcTimestamp());

               auto* behaviour = message.mutable_behaviour();
               behaviour->set_type(Behaviour::ACTION_STATE);
               auto* actionStateChange = behaviour->mutable_action_state_change();
               actionStateChange->set_state(ActionStateChange::KILL);
               actionStateChange->set_name(actionKill.name);
               for (auto& limbID : actionKill.limbs) {
                   actionStateChange->add_limbs(static_cast<int>(limbID));
               }

               send(message);
            });

            registerActionHandle = on<Trigger<RegisterAction>>([this] (const RegisterAction& action) {
                Message message;
                message.set_type(Message::BEHAVIOUR);
                message.set_utc_timestamp(getUtcTimestamp());

                auto* behaviour = message.mutable_behaviour();
                behaviour->set_type(Behaviour::ACTION_REGISTER);
                auto* actionRegister = behaviour->mutable_action_register();
                actionRegister->set_id(action.id);
                actionRegister->set_name(action.name);
                for(const auto& set : action.limbSet) {
                    auto* limbSet = actionRegister->add_limb_set();
                    limbSet->set_priority(set.first);
                    for (auto& limbID : set.second) {
                        limbSet->add_limbs(static_cast<int>(limbID));
                    }
                }

                send(message);
            });

            // This trigger gets the output from the sensors (unfiltered)
            sensorsHandle = on<Trigger<Sensors>, Options<Single, Priority<NUClear::LOW>>>([this](const Sensors& sensors) {

                Message message;

                message.set_type(Message::SENSOR_DATA);
                message.set_utc_timestamp(getUtcTimestamp());

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

            imageHandle = on<Trigger<Image>, Options<Single, Priority<NUClear::LOW>>>([this](const Image& image) {

                Message message;
                message.set_type(Message::IMAGE);
                message.set_utc_timestamp(getUtcTimestamp());

                auto* imageData = message.mutable_image();

                imageData->mutable_dimensions()->set_x(image.width());
                imageData->mutable_dimensions()->set_y(image.height());

                std::string* imageBytes = imageData->mutable_data();
                if(!image.source().empty()) {
                    imageData->set_format(messages::input::proto::Image::JPEG);

                    // Reserve enough space in the image data to store the output
                    imageBytes->reserve(image.source().size());

                    imageBytes->insert(imageBytes->begin(), std::begin(image.source()), std::end(image.source()));
                }
                else {
                    imageData->set_format(messages::input::proto::Image::YCbCr444);

                    imageBytes->reserve(image.raw().size() * sizeof(Image::Pixel));

                    imageBytes->insert(imageBytes->begin(), reinterpret_cast<const char*>(&image.raw().front()), reinterpret_cast<const char*>(&image.raw().back() + 1));
                }

                send(message);
            });

            reactionStatisticsHandle = on<Trigger<NUClear::ReactionStatistics>>([this](const NUClear::ReactionStatistics& stats) {
                Message message;
                message.set_type(Message::REACTION_STATISTICS);
                message.set_utc_timestamp(getUtcTimestamp());
                auto* reactionStatistics = message.mutable_reaction_statistics();
                //reactionStatistics->set_name(stats.name);
                reactionStatistics->set_reactionid(stats.reactionId);
                reactionStatistics->set_taskid(stats.taskId);
                reactionStatistics->set_causereactionid(stats.causeReactionId);
                reactionStatistics->set_causetaskid(stats.causeTaskId);
                reactionStatistics->set_emitted(duration_cast<microseconds>(stats.emitted.time_since_epoch()).count());
                reactionStatistics->set_started(duration_cast<microseconds>(stats.started.time_since_epoch()).count());
                reactionStatistics->set_finished(duration_cast<microseconds>(stats.finished.time_since_epoch()).count());
                reactionStatistics->set_name(stats.identifier[0]);
                reactionStatistics->set_triggername(stats.identifier[1]);
                reactionStatistics->set_functionname(stats.identifier[2]);

                send(message);
            });
            reactionStatisticsHandle.disable();

            classifiedImageHandle = on<Trigger<ClassifiedImage<ObjectClass>>, Options<Single, Priority<NUClear::LOW>>>([this](const ClassifiedImage<ObjectClass>& image) {

                Message message;
                message.set_type(Message::CLASSIFIED_IMAGE);
                message.set_utc_timestamp(getUtcTimestamp());

                auto* imageData = message.mutable_classified_image();

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
            });

            ballsHandle = on<Trigger<std::vector<VisionBall>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<VisionBall>& balls) {

                Message message;
                message.set_type(Message::VISION_OBJECT);
                message.set_utc_timestamp(getUtcTimestamp());

                auto* object = message.mutable_vision_object();
                object->set_type(messages::vision::proto::VisionObject::BALL);

                for(const auto& b : balls) {

                    auto* ball = object->add_ball();

                    auto* circle = ball->mutable_circle();
                    circle->set_radius(b.circle.radius);
                    circle->mutable_centre()->set_x(b.circle.centre[0]);
                    circle->mutable_centre()->set_y(b.circle.centre[1]);
                }

                send(message);

            });

            goalsHandle = on<Trigger<std::vector<VisionGoal>>, Options<Single, Priority<NUClear::LOW>>>([this] (const std::vector<VisionGoal>& goals) {

                Message message;
                message.set_type(Message::VISION_OBJECT);
                message.set_utc_timestamp(getUtcTimestamp());

                auto* object = message.mutable_vision_object();

                object->set_type(messages::vision::proto::VisionObject::GOAL);

                for(const auto& g : goals) {
                    auto* goal = object->add_goal();

                    goal->set_side(g.side == VisionGoal::Side::LEFT ? messages::vision::proto::VisionObject::Goal::LEFT
                                 : g.side == VisionGoal::Side::RIGHT ? messages::vision::proto::VisionObject::Goal::RIGHT
                                 : messages::vision::proto::VisionObject::Goal::UNKNOWN);

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
            });

            localisationHandle = on<Trigger<Every<100, std::chrono::milliseconds>>,
               With<Optional<std::vector<messages::localisation::Ball>>>,
               With<Optional<std::vector<messages::localisation::Self>>>,
               Options<Single>>("Localisation Reaction (NUbugger.cpp)",
                [this](const time_t&,
                       const std::shared_ptr<const std::vector<messages::localisation::Ball>>& opt_balls,
                       const std::shared_ptr<const std::vector<messages::localisation::Self>>& opt_robots) {
                auto robot_msg = std::make_unique<messages::localisation::FieldObject>();
                auto ball_msg = std::make_unique<messages::localisation::FieldObject>();
                bool robot_msg_set = false;
                bool ball_msg_set = false;

                if(opt_robots != nullptr && opt_robots->size() > 0) {
                    const auto& robots = *opt_robots;

                    // Robot message
                    std::vector<messages::localisation::FieldObject::Model> robot_msg_models;

                    for (auto& model : robots) {
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

                        // break; // Only output a single model
                    }
                    robot_msg->models = robot_msg_models;
                    robot_msg_set = true;
                }

                if(robot_msg_set && opt_balls != nullptr && opt_balls->size() > 0) {
                    const auto& balls = *opt_balls;
                    const auto& robots = *opt_robots;

                    arma::vec2 ball_pos = balls[0].position;

                    if (!balls[0].world_space) {
                        ball_pos = utility::localisation::transform::RobotToWorldTransform(
                        robots[0].position, robots[0].heading, balls[0].position);
                    }

                    // Ball message
                    std::vector<messages::localisation::FieldObject::Model> ball_msg_models;
                    
                    for (auto& model : balls) {
                        messages::localisation::FieldObject::Model ball_model;
                        ball_msg->name = "ball";
                        ball_model.wm_x = ball_pos[0];
                        ball_model.wm_y = ball_pos[1];
                        ball_model.heading = 0;
                        ball_model.sd_x = 0.1;
                        ball_model.sd_y = 0.1;

                        //Do we need to rotate the variances?
                        ball_model.sr_xx = model.sr_xx;
                        ball_model.sr_xy = model.sr_xy;
                        ball_model.sr_yy = model.sr_yy;
                        ball_model.lost = false;
                        ball_msg_models.push_back(ball_model);

                        // break; // Only output a single model
                    }
                    ball_msg->models = ball_msg_models;
                    ball_msg_set = true;
                }

                if (robot_msg_set || ball_msg_set)
                    EmitLocalisationModels(robot_msg, ball_msg);
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
            log("Received message of type:", message.type());
            switch (message.type()) {
                case Message::COMMAND:
                    recvCommand(message);
                    break;
                case Message::LOOKUP_TABLE:
                    recvLookupTable(message);
                    break;
                case Message::REACTION_HANDLERS:
                    recvReactionHandlers(message);
                    break;
                default:
                    return;
            }
        }

        void NUbugger::recvCommand(const Message& message) {
            std::string command = message.command().command();
            log("Received command:", command);
            if (command == "download_lut") {
                auto lut = powerplant.get<LookUpTable>();

                Message message;

                message.set_type(Message::LOOKUP_TABLE);
                message.set_utc_timestamp(getUtcTimestamp());

                auto* api_lookup_table = message.mutable_lookup_table();

                api_lookup_table->set_table(lut->getData());

                send(message);
            }
        }

        void NUbugger::recvLookupTable(const Message& message) {
            auto lookuptable = message.lookup_table();
            const std::string& lutData = lookuptable.table();

            log("Loading LUT");
            auto data = std::vector<char>(lutData.begin(), lutData.end());
            auto lut = std::make_unique<LookUpTable>(lookuptable.bits_y(), lookuptable.bits_cb(), lookuptable.bits_cr(), std::move(data));
            emit<Scope::DIRECT>(std::move(lut));

            if (lookuptable.save()) {
                log("Saving LUT to file");
                emit<Scope::DIRECT>(std::make_unique<SaveLookUpTable>());
            }
        }

        void NUbugger::recvReactionHandlers(const Message& message) {
            auto reactionHandlers = message.reactionhandlers();

            log("Reaction Handler Changes:");
            std::vector<std::tuple<bool, ReactionHandle, std::string>> handlers = {
                std::make_tuple(reactionHandlers.datapoints(), dataPointHandle, "Data Points"),
                std::make_tuple(reactionHandlers.actionstart(), actionStartHandle, "Action Start"),
                std::make_tuple(reactionHandlers.actionkill(), actionKillHandle, "Action Kill"),
                std::make_tuple(reactionHandlers.registeraction(), registerActionHandle, "Register Action"),
                std::make_tuple(reactionHandlers.sensors(), sensorsHandle, "Sensors"),
                std::make_tuple(reactionHandlers.image(), imageHandle, "Image"),
                std::make_tuple(reactionHandlers.reactionstatistics(), reactionStatisticsHandle, "Reaction Statistics"),
                std::make_tuple(reactionHandlers.classifiedimage(), classifiedImageHandle, "Classified Image"),
                std::make_tuple(reactionHandlers.goals(), goalsHandle, "Goals"),
                std::make_tuple(reactionHandlers.balls(), ballsHandle, "Balls"),
                std::make_tuple(reactionHandlers.localisation(), localisationHandle, "Localisation")
            };

            for (auto& handle : handlers) {
                bool enabled;
                ReactionHandle reactionHandle;
                std::string name;
                std::tie(enabled, reactionHandle, name) = handle;

                if (enabled) {
                    if (!reactionHandle.isEnabled()) {
                        reactionHandle.enable();
                        log(name, "Enabled");
                    }
                } else {
                    if (reactionHandle.isEnabled()) {
                        reactionHandle.disable();
                        log(name, "Disabled");
                    }
                }
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
            size_t messageSize = message.ByteSize();
            zmq::message_t packet(messageSize + 1);
            char* dataPtr = static_cast<char*>(packet.data());
            message.SerializeToArray(dataPtr + 1, messageSize);
            dataPtr[0] = message.type();
            send(packet);
        }

    } // support
} // modules
