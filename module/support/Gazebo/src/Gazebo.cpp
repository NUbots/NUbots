#include "Gazebo.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "extension/Configuration.h"

#include "message/input/Sensors.h"
#include "message/support/gazebo/GazeboBallLocation.h"
#include "message/support/gazebo/GazeboRobotLocation.h"
#include "message/support/gazebo/GazeboWorldCtrl.h"
#include "message/support/gazebo/GazeboWorldStatus.h"


#include "message/platform/darwin/DarwinSensors.h"
#include "utility/math/angle.h"

#include "utility/clock/CustomClock.hpp"
#include "utility/platform/darwin/DarwinSensors.h"

namespace module {
namespace support {

    using extension::Configuration;

    using message::input::Sensors;
    using message::motion::ServoTarget;
    using message::platform::darwin::DarwinSensors;
    using message::support::gazebo::GazeboBallLocation;
    using message::support::gazebo::GazeboRobotLocation;
    using message::support::gazebo::GazeboWorldCtrl;
    using message::support::gazebo::GazeboWorldStatus;

    Gazebo::Gazebo(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), node(std::make_unique<ignition::transport::Node>()) {

        utility::clock::custom_rtf = 1.0;

        on<Configuration>("Gazebo.yaml").then([this](const Configuration& config) {
            std::string p_uuid             = ignition::transport::Uuid().ToString();
            std::string n_uuid             = ignition::transport::Uuid().ToString();
            std::string topic_ctrl         = config["topic"]["NubotsIgusCtrl"].as<std::string>();
            std::string topic_status       = config["topic"]["NubotsIgusStatus"].as<std::string>();
            std::string topic_world_ctrl   = config["topic"]["NubotsWorldCtrl"].as<std::string>();
            std::string topic_world_status = config["topic"]["NubotsWorldStatus"].as<std::string>();
            std::string topic_ball_status  = config["topic"]["NubotsBallStatus"].as<std::string>();
            std::string host_addr          = config["address"]["host"].as<std::string>();
            std::string ctrl_addr          = config["address"]["ctrl"].as<std::string>();
            int gazebo_message_port        = config["gazebo"]["message_port"].as<int>();
            setenv("IGN_IP", host_addr.c_str(), 1);

            real_delta        = 0.0;
            sim_delta         = 0.0;
            current_real_time = 0.0;
            current_sim_time  = 0.0;

            connected = false;

            // Set up transport node for joint control
            // This will be ADVERTISED to the Ctrl topic
            ignition::transport::NodeOptions joint_ctrl_node_opts;
            joint_ctrl_node_opts.SetPartition(config["partition"]["joint"]);
            joint_ctrl_node_opts.SetNameSpace(config["ignition_namespace"]);
            joint_ctrl = std::make_unique<ignition::transport::Node>(joint_ctrl_node_opts);

            // Set up transport node for joint status
            // This will be SUBSCRIBED to the Status topic
            ignition::transport::NodeOptions joint_status_node_opts;
            joint_status_node_opts.SetPartition(config["partition"]["joint"]);
            joint_status_node_opts.SetNameSpace(config["ignition_namespace"]);
            joint_status = std::make_unique<ignition::transport::Node>(joint_status_node_opts);

            // Set up transport node for world control
            // This will be ADVERTISED to the Ctrl topic
            ignition::transport::NodeOptions world_ctrl_node_opts;
            world_ctrl_node_opts.SetPartition(config["partition"]["world"]);
            world_ctrl_node_opts.SetNameSpace(config["ignition_namespace"]);
            world_ctrl = std::make_unique<ignition::transport::Node>(world_ctrl_node_opts);

            // Set up transport node for world status
            // This will be SUBSCRIBED to the Status topic
            ignition::transport::NodeOptions world_status_node_opts;
            world_status_node_opts.SetPartition(config["partition"]["world"]);
            world_status_node_opts.SetNameSpace(config["ignition_namespace"]);
            world_status = std::make_unique<ignition::transport::Node>(world_status_node_opts);

            // Set up transport node for ball status
            // This will be SUBSCRIBED to the Status topic
            ignition::transport::NodeOptions ball_status_node_opts;
            ball_status_node_opts.SetPartition(config["partition"]["ball"]);
            ball_status_node_opts.SetNameSpace(config["ignition_namespace"]);
            ball_status = std::make_unique<ignition::transport::Node>(ball_status_node_opts);

            auto ad_msg_opts = std::make_unique<ignition::transport::AdvertiseMessageOptions>();

            discovery_node = std::make_unique<ignition::transport::MsgDiscovery>(p_uuid, gazebo_message_port);

            msg_publisher = std::make_unique<ignition::transport::MessagePublisher>(
                topic_ctrl, host_addr, ctrl_addr, p_uuid, n_uuid, "ADVERTISE", *ad_msg_opts);

            msg_pub_world = std::make_unique<ignition::transport::MessagePublisher>(
                topic_world_ctrl, host_addr, ctrl_addr, p_uuid, n_uuid, "ADVERTISE", *ad_msg_opts);

            bool joint_status_cb =
                joint_status->Subscribe<Gazebo, ignition::msgs::StringMsg>(topic_status, &Gazebo::JointStatusCB, this);

            bool world_status_cb = world_status->Subscribe<Gazebo, ignition::msgs::StringMsg>(
                topic_world_status, &Gazebo::WorldStatusCB, this);

            bool ball_status_cb = ball_status->Subscribe<Gazebo, ignition::msgs::StringMsg>(
                topic_ball_status, &Gazebo::BallStatusCB, this);

            // Set up a callback function for the discovery service
            discovery_node->ConnectionsCb([this](const ignition::transport::MessagePublisher& publisher) {
                connected = true;

                log<NUClear::DEBUG>(fmt::format("Discovered a Message Publisher! [{}]", publisher));
            });

            // Set up a callback function for the discovery service disconnections event
            discovery_node->DisconnectionsCb([this](const ignition::transport::MessagePublisher& publisher) {
                connected  = false;
                real_delta = 10000000000000.0;
                sim_delta  = 10000000000000.0;

                log<NUClear::DEBUG>(fmt::format("Disconnected from the Simulation! [{}]", publisher));
            });

            discovery_node->Start();

            if (!discovery_node->Advertise(*msg_publisher)) {
                log<NUClear::ERROR>("Failed to advertise the publisher node!");
            }

            if (!discovery_node->Advertise(*msg_pub_world)) {
                log<NUClear::ERROR>("Failed to advertise the worldPublisher node!");
            }

            if (!discovery_node->Discover(topic_status)) {
                log<NUClear::ERROR>("discovery of robot status topic failed...");
            }

            if (!discovery_node->Discover(topic_world_status)) {
                log<NUClear::ERROR>("discovery of world status topic failed...");
            }

            if (!discovery_node->Discover(topic_ball_status)) {
                log<NUClear::ERROR>("discovery of ball status topic failed...");
            }

            if (!joint_status_cb) {
                log<NUClear::ERROR>(fmt::format("Error subscribing to joint commands messages at [{}]", topic_status));
            }

            if (!world_status_cb) {
                log<NUClear::ERROR>(
                    fmt::format("Error subscribing to joint commands messages at [{}]", topic_world_status));
            }

            if (!ball_status_cb) {
                log<NUClear::ERROR>(
                    fmt::format("Error subscribing to joint commands messages at [{}]", topic_ball_status));
            }

            // ADVERTISE to the control topic
            pub = joint_ctrl->Advertise<ignition::msgs::StringMsg>(topic_ctrl);

            // ADVERTISE the world control topic
            world_pub = world_ctrl->Advertise<ignition::msgs::StringMsg>(topic_world_ctrl);
        });

        on<Trigger<std::vector<ServoTarget>>, With<DarwinSensors>>().then(
            [this](const std::vector<ServoTarget>& commands, const DarwinSensors& sensors) {
                if (!pub.Publish(parseServos(commands, sensors))) {
                    log<NUClear::ERROR>("Error publishing to topic [topicCtrl]");
                }
            });

        on<Trigger<GazeboWorldCtrl>>().then([this](const GazeboWorldCtrl& command) {
            ignition::msgs::StringMsg message;
            message.set_data(command.command);
            if (!world_pub.Publish(message)) {
                log<NUClear::ERROR>("Error publishing to world control topic!");
            }
        });
    }

    void Gazebo::JointStatusCB(const ignition::msgs::StringMsg& msg) {
        std::unique_ptr<DarwinSensors> sensors = std::make_unique<DarwinSensors>();
        std::stringstream ss(msg.data());
        std::string line;

        // Get remote name
        std::string source_name;
        std::getline(ss, source_name);
        remotes.insert(source_name);

        if (remotes.find(source_name) == remotes.begin()) {
            // Servos
            for (int i = 0; i < 20; ++i) {
                // Get a reference to the servo we are populating
                DarwinSensors::Servo& servo = utility::platform::darwin::getDarwinServo(i, *sensors);
                std::getline(ss, line);
                servo.presentSpeed = std::stof(line);
                std::getline(ss, line);
                servo.presentPosition = std::stof(line);
            }
            std::getline(ss, line);
            sensors->gyroscope.x = std::stof(line);
            std::getline(ss, line);
            sensors->gyroscope.y = std::stof(line);
            std::getline(ss, line);
            sensors->gyroscope.z = std::stof(line);

            std::getline(ss, line);
            sensors->accelerometer.x = std::stof(line);
            std::getline(ss, line);
            sensors->accelerometer.y = std::stof(line);
            std::getline(ss, line);
            sensors->accelerometer.z = std::stof(line);
            emit(sensors);
        }

        // Send Robot World coordinates
        std::unique_ptr<GazeboRobotLocation> location = std::make_unique<GazeboRobotLocation>();

        location->source_name = source_name;

        std::getline(ss, line);
        location->pos.x() = std::stof(line);
        std::getline(ss, line);
        location->pos.y() = std::stof(line);
        std::getline(ss, line);
        location->pos.z() = std::stof(line);

        emit(location);
    }

    void Gazebo::WorldStatusCB(const ignition::msgs::StringMsg& msg) {
        // GET WORLD updates sime time
        std::unique_ptr<GazeboWorldStatus> status = std::make_unique<GazeboWorldStatus>();
        std::stringstream ss(msg.data());
        std::string line;


        // Get remote name
        std::string source_name;
        std::getline(ss, source_name);
        remotes.insert(source_name);
        status->source_name = source_name;

        std::getline(ss, line);
        status->sim_time = std::stod(line);
        std::getline(ss, line);
        status->real_time      = std::stod(line);
        double prev_sim_delta  = sim_delta;
        double prev_real_delta = real_delta;

        real_delta = status->real_time - current_real_time;
        sim_delta  = status->sim_time - current_sim_time;

        // TODO: Confirm calculation simplification
        // utility::clock::custom_rtf =
        //  ((sim_delta + prev_sim_delta) / 2) / ((real_delta + prev_real_delta) / 2);
        utility::clock::custom_rtf = (sim_delta + prev_sim_delta) / (real_delta + prev_real_delta);
        current_real_time          = status->real_time;
        current_sim_time           = status->sim_time;
        emit(status);
    }

    void Gazebo::BallStatusCB(const ignition::msgs::StringMsg& msg) {
        // GET BALL updates
        std::unique_ptr<GazeboBallLocation> location = std::make_unique<GazeboBallLocation>();
        std::stringstream ss(msg.data());
        std::string line;

        // Get remote name
        std::string source_name;
        std::getline(ss, source_name);
        remotes.insert(source_name);
        location->source_name = source_name;

        std::getline(ss, line);
        location->pos.x() = std::stof(line);
        std::getline(ss, line);
        location->pos.y() = std::stof(line);
        std::getline(ss, line);
        location->pos.z() = std::stof(line);

        std::getline(ss, line);
        location->vel.x() = std::stof(line);
        std::getline(ss, line);
        location->vel.y() = std::stof(line);
        std::getline(ss, line);
        location->vel.z() = std::stof(line);

        emit(location);
    }

    const ignition::msgs::StringMsg Gazebo::GenerateMsg() {
        ignition::msgs::StringMsg message;
        message.set_data("SENDING\n");
        return message;
    }

    const ignition::msgs::StringMsg Gazebo::parseServos(const std::vector<ServoTarget>& commands,
                                                        const DarwinSensors& sensors) {
        ignition::msgs::StringMsg message;
        std::vector<int> command_order;
        std::string string = "";
        std::vector<int> command_present;
        std::vector<float> positions;
        std::vector<float> gains;
        std::vector<double> velocities;

        for (const auto& command : commands) {
            command_order.push_back(command.id);
        }

        for (int i = 0; i < 20; i++) {
            command_present.push_back(0);
            positions.push_back(0.0);
            gains.push_back(0.0);
            velocities.push_back(0.0);
        }

        for (int i = 0; i < commands.size(); i++) {
            for (int j = 0; j < commands.size(); j++) {
                if (command_order[j] == i) {
                    command_present[i]                = 1;
                    positions[i]                      = commands[j].position;
                    gains[i]                          = commands[j].gain;
                    NUClear::clock::duration duration = commands[j].time - NUClear::clock::now();
                    float diff                        = utility::math::angle::difference(
                        commands[j].position,
                        utility::platform::darwin::getDarwinServo(commands[j].id, sensors).presentPosition);
                    if (duration.count() > 0) {
                        velocities[i] = diff / ((double) duration.count() / (double) NUClear::clock::period::den);
                    }
                    else {
                        velocities[i] = 0.0;
                    }
                    break;
                }
            }
        }

        for (int i = 0; i < 20; i++) {
            if (command_present[i] == 1) {
                string = fmt::format("{}1\n{}\n{}\n{}\n", string, positions[i], gains[i], velocities[i]);
            }
            else {
                string = fmt::format("{}0\n0.11\n0.11\n0.11\n", string);
            }
        }
        message.set_data(string);
        return message;
    }
}  // namespace support
}  // namespace module
