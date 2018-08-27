#include "Gazebo.h"

#include "extension/Configuration.h"

#include "message/input/Sensors.h"
#include "message/support/Gazebo/GazeboMessage.h"


#include "message/platform/darwin/DarwinSensors.h"

#include "utility/platform/darwin/DarwinSensors.h"

namespace module {
namespace support {

    using extension::Configuration;
    using message::input::Sensors;
    using message::platform::darwin::DarwinSensors;
    using message::support::Gazebo::GazeboWorldCtrl;
    using message::support::Gazebo::GazeboBallLocation;
    using message::motion::ServoTarget;
    using namespace ignition;
    using namespace transport;

    Gazebo::Gazebo(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), node(new ignition::transport::Node()) {

        on<Configuration>("Gazebo.yaml").then([this](const Configuration& config) {
            setenv("IGN_PARTITION", "NubotsIgus", 1);
            setenv("IGN_IP", "10.1.0.92", 1);
            // gazebo_url  = config["gazebo"]["url"].as<std::string>();
            // gazebo_port = config["gazebo"]["port"].as<std::string>();
            static std::string pUuid = ignition::transport::Uuid().ToString();
            static std::string nUuid = ignition::transport::Uuid().ToString();
            static std::string topicStatus = "NubotsIgusStatus";
            static std::string topicCtrl = "NubotsIgusCtrl";
            static std::string topicWorldCtrl = "NubotsWorldCtrl";
            static std::string topicWorldStatus = "NubotsWorldStatus";
            static std::string hostAddr = "10.1.0.92";
            static std::string ctrlAddr = "10.1.0.92";
            static std::string id1 = "identity1";


            // Set up transport node for joint control
            // This will be ADVERTISED to the Ctrl topic
            ignition::transport::NodeOptions jointCtrlNodeOpts;
            jointCtrlNodeOpts.SetPartition("Joint");
            jointCtrlNodeOpts.SetNameSpace("Igus");
            jointCtrl = new ignition::transport::Node(jointCtrlNodeOpts);

            // Set up transport node for joint status
            // This will be SUBSCRIBED to the Status topic
            ignition::transport::NodeOptions jointStatusNodeOpts;
            jointStatusNodeOpts.SetPartition("Joint");
            jointStatusNodeOpts.SetNameSpace("Igus");
            jointStatus = new ignition::transport::Node(jointStatusNodeOpts);

            // Set up transport node for world control
            // This will be ADVERTISED to the Ctrl topic
            ignition::transport::NodeOptions worldCtrlNodeOpts;
            worldCtrlNodeOpts.SetPartition("World");
            worldCtrlNodeOpts.SetNameSpace("Igus");
            worldCtrl = new ignition::transport::Node(worldCtrlNodeOpts);

            // Set up transport node for joint status
            // This will be SUBSCRIBED to the Status topic
            ignition::transport::NodeOptions worldStatusNodeOpts;
            worldStatusNodeOpts.SetPartition("World");
            worldStatusNodeOpts.SetNameSpace("Igus");
            worldStatus = new ignition::transport::Node(worldStatusNodeOpts);

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

                    std::getline(ss, line);//log("gyroscope.x = " + line);
                    sensors->gyroscope.x = std::stof(line);
                    std::getline(ss, line);//log("gyroscope.y = " + line);
                    sensors->gyroscope.y = std::stof(line);
                    std::getline(ss, line);//log("gyroscope.z = " + line);
                    sensors->gyroscope.z = 0.0 - std::stof(line);

                    std::getline(ss, line);//log("accelerometer.x " + line);
                    sensors->accelerometer.x = std::stof(line);
                    std::getline(ss, line);//log("accelerometer.y " + line);
                    sensors->accelerometer.y = 0.0 - std::stof(line);
                    std::getline(ss, line);//log("accelerometer.z " + line);
                    sensors->accelerometer.z = 0.0 - std::stof(line);

                    emit(sensors);

                    std::unique_ptr<GazeboBallLocation> ballLoc = std::make_unique<GazeboBallLocation>();
                    std::getline(ss, line);
                    ballLoc->x = std::stof(line);
                    std::getline(ss, line);
                    ballLoc->y = std::stof(line);
                    std::getline(ss, line);
                    ballLoc->z = std::stof(line);
                    emit(ballLoc);
                }
            );

            std::function<void(const ignition::msgs::StringMsg &_msg)> WorldStatusCb(
            [this](const ignition::msgs::StringMsg &_msg) -> void
                {
                    // GET WORLD updates sime time
                }
            );

            // Set up a callback function for the discovery service
            std::function<void(const ignition::transport::MessagePublisher &_publisher)> onDiscoveryCb(
            [this](const ignition::transport::MessagePublisher &_publisher) -> void
                {
                    //std::cout << "Discovered a Message Publisher!" << std::endl;
                    //std::cout << _publisher << std::endl;
                });

            // Set up a callback function for the discovery service disconnections event
            std::function<void(const ignition::transport::MessagePublisher &_publisher)> onDisconnectionCb(
                [this](const ignition::transport::MessagePublisher &_publisher) -> void
                {
                    //std::cout << "Disconnected from the Simulation!" << std::endl;
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

            if (!jointStatus->Subscribe<ignition::msgs::StringMsg>(topicStatus, JointStatusCb))
                std::cout << "Error subscribing to joint commands messages at [" << topicStatus << "]" << std::endl;

            if (!worldStatus->Subscribe<ignition::msgs::StringMsg>(topicWorldStatus, WorldStatusCb))
                std::cout << "Error subscribing to joint commands messages at [" << worldStatus << "]" << std::endl;

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

        on<Trigger<std::vector<ServoTarget>>>().then(
            [this](const std::vector<ServoTarget>& commands) {

            if (!this->pub.Publish(this->parseServos(commands)))
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

    const ignition::msgs::StringMsg Gazebo::parseServos(const std::vector<ServoTarget>& commands)
    {
        ignition::msgs::StringMsg message;
        std::vector<int> commandOrder;
        std::string string = "";
        std::vector<int> commandPresent;
        std::vector<float> positions;
        std::vector<float> gains;

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
            }
            else
            {
                string += std::to_string(0) + "\n";
                string += std::to_string((float) 0.11) + "\n";
                string += std::to_string((float) 0.11) + "\n";
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
