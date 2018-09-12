#include "Gazebo.h"

#include "extension/Configuration.h"

#include "message/input/Sensors.h"
#include "message/support/Gazebo/GazeboWorldCtrl.h"
#include "message/support/Gazebo/GazeboWorldStatus.h"
#include "message/support/Gazebo/GazeboBallLocation.h"
#include "message/support/Gazebo/GazeboRobotLocation.h"


#include "utility/math/angle.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/platform/darwin/DarwinSensors.h"
//#include "utility/clock/CustomClock.hpp"

namespace module {
namespace support {

    using extension::Configuration;
    using message::input::Sensors;
    using message::platform::darwin::DarwinSensors;
    using message::support::Gazebo::GazeboWorldCtrl;
    using message::support::Gazebo::GazeboWorldStatus;
    using message::support::Gazebo::GazeboBallLocation;
    using message::support::Gazebo::GazeboRobotLocation;
    using message::motion::ServoTarget;
    using namespace ignition;
    using namespace transport;

    Gazebo::Gazebo(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), node(new ignition::transport::Node()) {

        //utility::clock::custom_rtf = 1.0;
        //utility::clock::lastUpdate = std::chrono::steady_clock::now();
        //log(utility::clock::start);


        on<Configuration>("Gazebo.yaml").then([this](const Configuration& config) {
            //setenv("IGN_PARTITION", "Nubots", 1);
            setenv("IGN_IP", "10.1.0.92", 1);
            // gazebo_url  = config["gazebo"]["url"].as<std::string>();
            // gazebo_port = config["gazebo"]["port"].as<std::string>();
            static std::string pUuid = ignition::transport::Uuid().ToString();
            static std::string nUuid = ignition::transport::Uuid().ToString();
            static std::string topicCtrl = "NubotsIgusCtrl";
            static std::string topicStatus = "NubotsIgusStatus";
            static std::string topicWorldCtrl = "NubotsWorldCtrl";
            static std::string topicWorldStatus = "NubotsWorldStatus";
            static std::string topicBallStatus = "NubotsBallStatus";
            static std::string hostAddr = "10.1.0.92";
            static std::string ctrlAddr = "10.1.0.92";

            this->realDelta = 0.0;
            this->simDelta = 0.0;
            this->currentRealTime = 0.0;
            this->currentSimTime = 0.0;

            this->connected = false;

            // Set up transport node for joint control
            // This will be ADVERTISED to the Ctrl topic
            ignition::transport::NodeOptions jointCtrlNodeOpts;
            jointCtrlNodeOpts.SetPartition("Igus");
            jointCtrlNodeOpts.SetNameSpace("Nubots");
            jointCtrl = new ignition::transport::Node(jointCtrlNodeOpts);

            // Set up transport node for joint status
            // This will be SUBSCRIBED to the Status topic
            ignition::transport::NodeOptions jointStatusNodeOpts;
            jointStatusNodeOpts.SetPartition("Igus");
            jointStatusNodeOpts.SetNameSpace("Nubots");
            jointStatus = new ignition::transport::Node(jointStatusNodeOpts);

            // Set up transport node for world control
            // This will be ADVERTISED to the Ctrl topic
            ignition::transport::NodeOptions worldCtrlNodeOpts;
            worldCtrlNodeOpts.SetPartition("World");
            worldCtrlNodeOpts.SetNameSpace("Nubots");
            worldCtrl = new ignition::transport::Node(worldCtrlNodeOpts);

            // Set up transport node for world status
            // This will be SUBSCRIBED to the Status topic
            ignition::transport::NodeOptions worldStatusNodeOpts;
            worldStatusNodeOpts.SetPartition("World");
            worldStatusNodeOpts.SetNameSpace("Nubots");
            worldStatus = new ignition::transport::Node(worldStatusNodeOpts);

            // Set up transport node for ball status
            // This will be SUBSCRIBED to the Status topic
            ignition::transport::NodeOptions ballStatusNodeOpts;
            ballStatusNodeOpts.SetPartition("Ball");
            ballStatusNodeOpts.SetNameSpace("Nubots");
            ballStatus = new ignition::transport::Node(ballStatusNodeOpts);

            static const ignition::transport::AdvertiseMessageOptions* AdMsgOpts =
                new ignition::transport::AdvertiseMessageOptions();

            discoveryNode = new MsgDiscovery(pUuid, g_msgPort);

            msgPublisher = new MessagePublisher(topicCtrl, hostAddr, ctrlAddr, pUuid,
                nUuid, "ADVERTISE", *AdMsgOpts);

            msgPubWorld = new MessagePublisher(topicWorldCtrl, hostAddr, ctrlAddr, pUuid,
                nUuid, "ADVERTISE", *AdMsgOpts);

            std::function<void(const ignition::msgs::StringMsg &_msg)> JointStatusCb(
            [this](const ignition::msgs::StringMsg &_msg) -> void
                {
                    std::unique_ptr<DarwinSensors> sensors = std::make_unique<DarwinSensors>();
                    std::stringstream ss(_msg.data());
                    std::string line;

                    // Servos
                    for (int i = 0; i < 20; ++i)
                    {
                        // Get a reference to the servo we are populating
                        DarwinSensors::Servo& servo = utility::platform::darwin::getDarwinServo(i, *sensors);
                        std::getline(ss, line);
                        servo.presentSpeed    = std::stof(line);
                        std::getline(ss, line);
                        servo.presentPosition = std::stof(line);
                    }

                    //std::getline(ss, line);
                    //std::cout << line << std::endl;

                    // Timestamp when our data was taken
                    //sensors.timestamp = std::chrono::duration; // NUClear::clock::now();

                    /*sensors->gyroscope.x = 0.0;
                    sensors->gyroscope.y = 0.0;
                    sensors->gyroscope.x = 0.0;

                    sensors->accelerometer.x = 0.0;
                    sensors->accelerometer.y = 0.0;
                    sensors->accelerometer.z = -9.81;*/

                    std::getline(ss, line);//log("gyroscope.x =     " + line);
                    sensors->gyroscope.x = std::stof(line);
                    std::getline(ss, line);//log("gyroscope.y =     " + line);
                    sensors->gyroscope.y = std::stof(line);
                    std::getline(ss, line);//log("gyroscope.z =     " + line);
                    sensors->gyroscope.z = std::stof(line);

                    std::getline(ss, line);//log("accelerometer.x = " + line);
                    sensors->accelerometer.x = std::stof(line);
                    std::getline(ss, line);//log("accelerometer.y = " + line);
                    sensors->accelerometer.y = std::stof(line);
                    std::getline(ss, line);//log("accelerometer.z = " + line);
                    sensors->accelerometer.z = std::stof(line);
                    emit(sensors);

                    // Send Robot World coordinates
                    std::unique_ptr<GazeboRobotLocation> location = std::make_unique<GazeboRobotLocation>();

                    std::getline(ss, line);
                    location->x = std::stof(line);
                    std::getline(ss, line);
                    location->y = std::stof(line);
                    std::getline(ss, line);
                    location->z = std::stof(line);

                    //std::cout << "emitting ball location..." << std::endl;
                    emit(location);
                }
            );

            std::function<void(const ignition::msgs::StringMsg &_msg)> WorldStatusCb(
            [this](const ignition::msgs::StringMsg &_msg) -> void
                {
                    // GET WORLD updates sime time
                    std::unique_ptr<GazeboWorldStatus> status = std::make_unique<GazeboWorldStatus>();
                    std::stringstream ss(_msg.data());
                    std::string line;

                    std::getline(ss, line);
                    status->simTime = std::stod(line);
                    std::getline(ss, line);
                    status->realTime = std::stod(line);
                    double prevSimDelta = simDelta;
                    double prevRealDelta = realDelta;

                    realDelta = status->realTime - currentRealTime;
                    simDelta = status->simTime - currentSimTime;


                    //utility::clock::custom_rtf = ((simDelta + prevSimDelta) / 2) / ((realDelta + prevRealDelta) / 2);//log(utility::clock::custom_rtf);
                    //utility::clock::lastUpdate = std::chrono::time_point::now();
                    //std::cout << (simDelta / realDelta) << std::endl;
                    currentRealTime = status->realTime;
                    currentSimTime = status->simTime;
                    emit(status);
                }
            );

            std::function<void(const ignition::msgs::StringMsg &_msg)> BallStatusCb(
            [this](const ignition::msgs::StringMsg &_msg) -> void
                {
                    // GET BALL updates
                    std::unique_ptr<GazeboBallLocation> location = std::make_unique<GazeboBallLocation>();
                    std::stringstream ss(_msg.data());
                    std::string line;

                    std::getline(ss, line);
                    location->x = std::stof(line);
                    std::getline(ss, line);
                    location->y = std::stof(line);
                    std::getline(ss, line);
                    location->z = std::stof(line);

                    std::getline(ss, line);
                    location->velx = std::stof(line);
                    std::getline(ss, line);
                    location->vely = std::stof(line);
                    std::getline(ss, line);
                    location->velz = std::stof(line);

                    //std::cout << "emitting ball location..." << std::endl;
                    emit(location);
                }
            );

            // Set up a callback function for the discovery service
            std::function<void(const ignition::transport::MessagePublisher &_publisher)> onDiscoveryCb(
            [this](const ignition::transport::MessagePublisher &_publisher) -> void
                {
                    this->connected = true;

                    std::cout << "Discovered a Message Publisher!" << std::endl;
                    //std::cout << _publisher << std::endl;
                });

            // Set up a callback function for the discovery service disconnections event
            std::function<void(const ignition::transport::MessagePublisher &_publisher)> onDisconnectionCb(
                [this](const ignition::transport::MessagePublisher &_publisher) -> void
                {
                    this->connected = false;
                    this->realDelta = 10000000000000.0;
                    this->simDelta = 10000000000000.0;

                    std::cout << "Disconnected from the Simulation!" << std::endl;
                    //std::cout << _publisher << std::endl;
                });

            discoveryNode->ConnectionsCb(onDiscoveryCb);
            discoveryNode->DisconnectionsCb(onDisconnectionCb);

            discoveryNode->Start();

            if (!discoveryNode->Advertise(*msgPublisher))
                std::cout << "Failed to advertise the publisher node!" << std::endl;

            if (!discoveryNode->Advertise(*msgPubWorld))
                std::cout << "Failed to advertise the worldPublisher node!" << std::endl;

            if (!discoveryNode->Discover(topicStatus))
                std::cout << "discovery of robot status topic failed..." << std::endl;

            if (!discoveryNode->Discover(topicWorldStatus))
                std::cout << "discovery of world status topic failed..." << std::endl;

            if (!discoveryNode->Discover(topicBallStatus))
                std::cout << "discovery of ball status topic failed..." << std::endl;

            if (!jointStatus->Subscribe<ignition::msgs::StringMsg>(topicStatus, JointStatusCb))
                std::cout << "Error subscribing to joint commands messages at [" << topicStatus << "]" << std::endl;

            if (!worldStatus->Subscribe<ignition::msgs::StringMsg>(topicWorldStatus, WorldStatusCb))
                std::cout << "Error subscribing to joint commands messages at [" << topicWorldStatus << "]" << std::endl;

            if (!ballStatus->Subscribe<ignition::msgs::StringMsg>(topicBallStatus, BallStatusCb))
                std::cout << "Error subscribing to joint commands messages at [" << topicBallStatus << "]" << std::endl;

            // ADVERTISE to the control topic
            pub = jointCtrl->Advertise<ignition::msgs::StringMsg>(topicCtrl);

            // ADVERTISE the world control topic
            worldPub = worldCtrl->Advertise<ignition::msgs::StringMsg>(topicWorldCtrl);
        });

        on<Startup>().then([this]() {

        });

        on<Shutdown>().then([this]() {

        });

        /*on<Every<1, Per<std::chrono::seconds>>>().then([this]() {
            //NUClear::log(__LINE__);
            if (pub)
            {
            //NUClear::log(__LINE__);
                if (!pub.Publish())
                    std::cout << "Error publishing to topic [topicCtrl]" << std::endl;
                else std::cout << "SENDING\n";
            //NUClear::log(__LINE__);

            //NUClear::log(__LINE__);
                if (pub.HasConnections())
                    std::cout << "Has connections!" << std::endl;
                else
                    std::cout << "No connections..." << std::endl;
            //NUClear::log(__LINE__);
            }

        });*/

        on<Trigger<std::vector<ServoTarget>>, With<DarwinSensors>>().then(
            [this](const std::vector<ServoTarget>& commands, const DarwinSensors& sensors) {

            if (!this->pub.Publish(this->parseServos(commands, sensors)))
                std::cout << "Error publishing to topic [topicCtrl]" << std::endl;
        });

        on<Trigger<GazeboWorldCtrl>>().then(
            [this](const GazeboWorldCtrl& command) {
            ignition::msgs::StringMsg message;
            message.set_data(command.command);
            if (!this->worldPub.Publish(message))
                std::cout << "Error publishing to world control topic!" << std::endl;
        });
    }

    const ignition::msgs::StringMsg Gazebo::GenerateMsg()
    {
        ignition::msgs::StringMsg message;
        std::string string = "SENDING\n";
        message.set_data(string);
        return message;
    }

    const ignition::msgs::StringMsg Gazebo::parseServos(const std::vector<ServoTarget>& commands, const DarwinSensors& sensors)
    {
        ignition::msgs::StringMsg message;
        std::vector<int> commandOrder;
        std::string string = "";
        std::vector<int> commandPresent;
        std::vector<float> positions;
        std::vector<float> gains;
        std::vector<double> velocities;

        for (const auto& command : commands)
        {
            commandOrder.push_back(command.id);
            //if (command.id == 12)
            //{
            //    log("pos:  ");
            //    log(command.position);
            //    log("gain: ");
            //    log(command.gain);
            //}
            //if (command.id == 13)
            //{
            //    log("13 pos:  ");
            //    log(command.position);
            //    log("13 gain: ");
            //    log(command.gain);
            //}
        }

        for (int i = 0; i < 20; i++)
        {
            commandPresent.push_back(0);
            positions.push_back(0.0);
            gains.push_back(0.0);
            velocities.push_back(0.0);
        }

        for (int i = 0; i < commands.size(); i++)
        {
            for (int j = 0; j < commands.size(); j++)
            {
                if (commandOrder[j] == i)
                {
                    commandPresent[i] = 1;
                    positions[i] = commands[j].position;
                    gains[i] = commands[j].gain;
                    NUClear::clock::duration duration = commands[j].time - NUClear::clock::now();
                    float diff = utility::math::angle::difference(
                        commands[j].position, utility::platform::darwin::getDarwinServo(commands[j].id, sensors).presentPosition);
                    if (duration.count() > 0)
                        velocities[i] = diff / ((double)duration.count() / (double)NUClear::clock::period::den);
                    else
                        velocities[i] = 0.0;
                    break;
                }
            }
        }

        for (int i = 0; i < 20; i++)
        {
            if (commandPresent[i] == 1)
            {
                string += std::to_string(1) + "\n";
                string += std::to_string(positions[i]) + "\n";
                string += std::to_string(gains[i]) + "\n";
                string += std::to_string(velocities[i]) + "\n";
            }
            else
            {
                string += std::to_string(0) + "\n";
                string += std::to_string((float) 0.11) + "\n";
                string += std::to_string((float) 0.11) + "\n";
                string += std::to_string((double) 0.11) + "\n";
            }
        }
        message.set_data(string);
        return message;
    }
}
}

/*
#include <chrono>

#define NUCLEAR_CLOCK struct clock { \
    using duration              = std::chrono::steady_clock::duration; \
    using rep                   = std::chrono::steady_clock::rep; \
    using period                = std::chrono::steady_clock::period; \
    using time_point            = std::chrono::steady_clock::time_point; \
    static constexpr bool is_steady = false; \
    static time_point timestamp; \
    static inline time_point now() noexcept { \
      return timestamp; \
    } \
}; \
clock::time_point clock::timestamp;

#include <nuclear>
#include <iostream>

int main(void) {
  for (int i = 0; i < 10; i++){
    std::cout << NUClear::clock::now().time_since_epoch().count() << std::endl;
    NUClear::clock::timestamp = std::chrono::steady_clock::now();
  }
  return 0;*/
