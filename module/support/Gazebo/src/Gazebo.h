#ifndef MODULE_SUPPORT_GAZEBO_H
#define MODULE_SUPPORT_GAZEBO_H

#include <nuclear>

//#include <ignition/msgs1/ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>
//#include <gazebo/gazebo_config.h>
//#include <gazebo/msgs/msgs.hh>
//#include <gazebo/transport/transport.hh>

// ZeroMQ_libzmq _VERSION:INTERNAL= git add . / git commit -am "message" git push
//ADVANCED       ^ space breaks ./b module generate   property for variable: _GFORTRAN_EXECUTABLE


#include "message/motion/ServoTarget.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/platform/darwin/DarwinSensors.h"

namespace module {
namespace support {

    class Gazebo : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the gazebo reactor.
        explicit Gazebo(std::unique_ptr<NUClear::Environment> environment);

        ignition::transport::MsgDiscovery* discoveryNode;
        ignition::transport::MessagePublisher* msgPublisher;
        ignition::transport::MessagePublisher* msgPubWorld;
        ignition::transport::Node* node;
        ignition::transport::Node* jointCtrl;
        ignition::transport::Node* jointStatus;
        ignition::transport::Node* worldCtrl;
        ignition::transport::Node* worldStatus;
        ignition::transport::Node* ballStatus;
        ignition::transport::Node* spareNode;
        ignition::transport::Node::Publisher pub;
        ignition::transport::Node::Publisher worldPub;
        const ignition::msgs::StringMsg parseServos(const std::vector<message::motion::ServoTarget>& commands, const message::platform::darwin::DarwinSensors& sensors);
        const ignition::msgs::StringMsg GenerateMsg();
        static const int g_msgPort = 11319;

        double currentRealTime;
        double currentSimTime;
        double realDelta;
        double simDelta;

        bool connected;
    private:

    };
}
}

#endif  // MODULE_SUPPORT_GAZEBO_H
