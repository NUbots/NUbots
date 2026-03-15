#ifndef MODULE_NETWORK_NETWORK_FORWARDER_HPP
#define MODULE_NETWORK_NETWORK_FORWARDER_HPP

#include <chrono>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <nuclear>
#include <string>
#include <utility>

#include "Forwarder.hpp"

namespace module::network {

    class NetworkForwarder : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the NetworkForwarder reactor.
        explicit NetworkForwarder(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// The currently active message forwarders mapped by message name then mapped by NUClearNet client name
        std::map<std::string, std::map<std::string, std::unique_ptr<Forwarder>, std::less<>>, std::less<>> forwarders;
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_NETWORK_FORWARDER_HPP
