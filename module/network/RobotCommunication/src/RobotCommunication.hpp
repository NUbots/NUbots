#ifndef MODULE_NETWORK_ROBOTCOMMUNICATION_HPP
#define MODULE_NETWORK_ROBOTCOMMUNICATION_HPP

#include <nuclear>
#include <string>

namespace module::network {

    class RobotCommunication : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The port to communicate over
            uint send_port = 0;
            /// @brief The port to communicate over (should match receive port)
            uint receive_port = 0;
            /// @brief The IP address used for broadcasting data
            std::string broadcast_ip = "";
        } cfg;

        /// @brief Handle for incoming UDP message. This will be bound/unbound during (re)connection
        ReactionHandle listen_handle;


    public:
        /// @brief Called by the powerplant to build and setup the RobotCommunication reactor.
        explicit RobotCommunication(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_ROBOTCOMMUNICATION_HPP
