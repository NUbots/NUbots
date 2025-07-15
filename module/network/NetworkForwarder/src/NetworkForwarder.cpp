#include "NetworkForwarder.hpp"

#include <chrono>
#include <fmt/format.h>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <tuple>
#include <utility>

#include "Forwarder.hpp"

#include "extension/Configuration.hpp"

#include "message/eye/Rpc.hpp"
#include "message/network/MessageRequest.hpp"
#include "message/network/NetworkForwarderConfig.hpp"
#include "message/reflection.hpp"

#include "utility/support/math_string.hpp"

namespace module::network {

    using extension::Configuration;
    using message::eye::RpcResponseMeta;
    using message::network::MessageRequest;
    using message::network::NetworkForwarderConfig;
    using NetworkForwarderTarget = NetworkForwarderConfig::Target;
    using MessageConfig          = NetworkForwarderConfig::MessageConfig;
    using NUClear::message::NetworkJoin;

    namespace {

        /**
         * Parse a send configuration string from the YAML file
         *
         * @param value The send configuration string from the YAML file
         *
         * @return A tuple containing (enabled, reliable, rate_limit)
         */
        std::tuple<bool, bool, double> parse_send_config(const std::string& value) {

            // Check if forwarding is disabled
            static const std::regex false_re("n|N|no|No|NO|false|False|FALSE|off|Off|OFF");
            if (std::regex_match(value, false_re)) {
                // Disable sending
                return std::make_tuple(false, false, 0.0);
            }

            // Check if reliable forwarding is enabled
            static const std::regex reliable_re("reliable|Reliable|RELIABLE");
            if (std::regex_match(value, reliable_re)) {
                return std::make_tuple(true, true, 0.0);
            }

            // Check if normal forwarding is enabled with no rate limit
            static const std::regex true_re("y|Y|yes|Yes|YES|true|True|TRUE|on|On|ON");
            if (std::regex_match(value, true_re)) {
                return std::make_tuple(true, false, 0.0);
            }

            // If none of the above, parse the value as a rate limit (messages per second)
            auto rate_limit = utility::support::parse_math_string<double>(value);
            return std::make_tuple(true, false, rate_limit);
        }

    }  // namespace

    NetworkForwarder::NetworkForwarder(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<NetworkForwarderConfig>>().then([this](const NetworkForwarderConfig& cfg) {
            // First, remove any forwarders whose targets no longer exist
            std::erase_if(forwarders, [&cfg](const auto& item) {
                const auto& [target_name, _] = item;
                return !cfg.targets.contains(target_name);
            });

            // Process each target
            for (const auto& [target_name, target_cfg] : cfg.targets) {
                auto& target_forwarders = forwarders[target_name];

                // Get all message configurations from the target
                const auto& message_configs = target_cfg.messages;

                // Remove forwarders for messages no longer in the config or with both send/recv disabled
                std::erase_if(target_forwarders, [&message_configs](const auto& item) {
                    const auto& [message_name, _] = item;
                    if (!message_configs.contains(message_name)) {
                        return true;
                    }
                    const auto& config = message_configs.at(message_name);
                    return !config.send && !config.recv;
                });

                // Create or update forwarders for configured messages with at least one direction enabled
                for (const auto& [message_name, config] : message_configs) {
                    // If either direction is enabled
                    if (config.send || config.recv) {
                        try {
                            if (!target_forwarders.contains(message_name)) {
                                target_forwarders[message_name] = Forwarder::create(*this, target_name, message_name);
                            }
                            target_forwarders[message_name]->configure(config);
                        }
                        catch (std::exception& e) {
                            log<ERROR>("Error creating forwarder for message", message_name, ":", e.what());
                        }
                    }
                }
            }
        });

        on<Configuration>("NetworkForwarder.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            // Build the NetworkForwarderConfig from the YAML configuration
            auto network_config = std::make_unique<NetworkForwarderConfig>();

            // Process each target in the YAML
            for (const auto& target : cfg["targets"]) {
                auto target_name      = target.first.as<std::string>();
                auto& message_configs = network_config->targets[target_name];

                // Process send configuration
                for (const auto& send : target.second["send"]) {
                    auto message_name = send.first.as<std::string>();
                    auto config_value = send.second.as<std::string>();

                    // Create a new config or get existing one
                    auto& config = message_configs.messages[message_name];

                    // Parse the send configuration and apply the results
                    std::tie(config.send, config.reliable, config.rate_limit) = parse_send_config(config_value);
                }

                // Process receive configuration
                for (const auto& recv : target.second["recv"]) {
                    auto message_name                           = recv.first.as<std::string>();
                    message_configs.messages[message_name].recv = recv.second.as<bool>();
                }
            }

            emit<Scope::INLINE>(std::move(network_config));
        });

        on<Trigger<NUClear::message::NetworkJoin>, Sync<NetworkForwarder>>().then([this](const NetworkJoin& remote) {
            const auto& target_name = remote.name;

            // If the remote is a target we are forwarding to, send it the most recent version of all the messages
            if (forwarders.contains(target_name)) {
                for (auto& [message_name, forwarder] : forwarders.at(target_name)) {
                    forwarder->send_out(0);
                }
            }
        });

        on<Network<MessageRequest>, Sync<NetworkForwarder>>().then([this](const NetworkSource& target,
                                                                          const MessageRequest& request) {
            const auto& message_name = request.name;
            const auto& target_name  = target.name;

            std::string error_message;
            if (forwarders.contains(target_name) && forwarders.at(target_name).contains(message_name)) {
                try {
                    forwarders.at(target_name).at(message_name)->send_out(request.subtype);
                }
                catch (std::exception& e) {
                    error_message = e.what();
                }
            }
            else {
                error_message = fmt::format("{} is not configured to be forwarded to {}", message_name, target_name);
            }

            if (!error_message.empty()) {
                log<ERROR>("Error processing message request:", error_message);
                emit<Scope::NETWORK>(std::make_unique<RpcResponseMeta>(request.rpc.token, false, error_message));
            }
            else {
                emit<Scope::NETWORK>(std::make_unique<RpcResponseMeta>(request.rpc.token, true, ""));
            }
        });
    }

}  // namespace module::network
