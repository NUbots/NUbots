#include "Forwarder.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "MessageForwarder.hpp"
#include "NetworkForwarder.hpp"

#include "message/reflection.hpp"

namespace module::network {

    std::unique_ptr<Forwarder> Forwarder::create(NetworkForwarder& reactor,
                                                 const std::string& target,
                                                 const std::string& message_name) {
        // Create a forwarder for this message type
        return message::reflection::from_string<MessageForwarder>(message_name, reactor, target, message_name);
    }

}  // namespace module::network
