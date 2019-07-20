#ifndef MODULE_SUPPORT_GAZEBO_H
#define MODULE_SUPPORT_GAZEBO_H

#include <memory>
#include <nuclear>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "message/motion/ServoTarget.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/platform/darwin/DarwinSensors.h"

namespace module {
namespace support {

    class Gazebo : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the gazebo reactor.
        explicit Gazebo(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::unique_ptr<ignition::transport::MsgDiscovery> discovery_node;
        std::unique_ptr<ignition::transport::MessagePublisher> msg_publisher;
        std::unique_ptr<ignition::transport::MessagePublisher> msg_pub_world;
        std::unique_ptr<ignition::transport::Node> node;
        std::unique_ptr<ignition::transport::Node> joint_ctrl;
        std::unique_ptr<ignition::transport::Node> joint_status;
        std::unique_ptr<ignition::transport::Node> world_ctrl;
        std::unique_ptr<ignition::transport::Node> world_status;
        std::unique_ptr<ignition::transport::Node> ball_status;
        std::unique_ptr<ignition::transport::Node> spare_node;
        ignition::transport::Node::Publisher pub;
        ignition::transport::Node::Publisher world_pub;

        void JointStatusCB(const ignition::msgs::StringMsg& msg);
        void WorldStatusCB(const ignition::msgs::StringMsg& msg);
        void BallStatusCB(const ignition::msgs::StringMsg& msg);

        const ignition::msgs::StringMsg parseServos(const std::vector<message::motion::ServoTarget>& commands,
                                                    const message::platform::darwin::DarwinSensors& sensors);
        const ignition::msgs::StringMsg GenerateMsg();

        double current_real_time;
        double current_sim_time;
        double real_delta;
        double sim_delta;

        bool connected;

        // Remote simulators
        std::unordered_set<std::string> remotes;
    };
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_GAZEBO_H
