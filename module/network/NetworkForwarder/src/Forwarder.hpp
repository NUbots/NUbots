#ifndef MODULE_NETWORK_NETWORK_FORWARDER_FORWARDER_HPP
#define MODULE_NETWORK_NETWORK_FORWARDER_FORWARDER_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "message/network/NetworkForwarderConfig.hpp"

namespace module::network {

    class NetworkForwarder;

    /**
     * @brief Abstract base class for forwarding messages between local and network scopes
     */
    class Forwarder {
    public:
        /// Configuration parameters for message forwarding
        using Config = message::network::NetworkForwarderConfig::MessageConfig;

        virtual ~Forwarder() = default;

        /**
         * @return The fully qualified name of the message type being forwarded
         */
        [[nodiscard("getter")]] virtual std::string message_name() const = 0;

        /**
         * @return The target name for this forwarder
         */
        [[nodiscard("getter")]] virtual std::string target_name() const = 0;

        /**
         * @brief Configure the forwarder with new settings
         *
         * @param new_config The configuration settings for the forwarder
         */
        virtual void configure(const Config& new_config) = 0;

        /**
         * @brief Send the most recent local message as a network message
         *
         * @param id The ID of the message subtype to send, or 0 to send all subtypes
         */
        virtual void send_out(const uint32_t& id) = 0;

        /**
         * @brief Creates a new forwarder for the specified message type with everything disabled by default
         *
         * @param reactor The NetworkForwarder reactor that will handle the forwarding
         * @param target The name of the NUClearNet client to forward messages to and/or from
         * @param message_name The fully qualified name of the message type to forward
         *
         * @return A unique pointer to the created forwarder
         */
        static std::unique_ptr<Forwarder> create(NetworkForwarder& reactor,
                                                 const std::string& target,
                                                 const std::string& message_name);
    };

}  // namespace module::network

#endif  // MODULE_NETWORK_NETWORK_FORWARDER_FORWARDER_HPP
