#include "NetworkForwarder.hpp"

#include "extension/Configuration.hpp"

#include "utility/support/yaml_expression.hpp"

namespace module::network {

    using extension::Configuration;
    using utility::support::Expression;

    NetworkForwarder::NetworkForwarder(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Register the message forwarding
        register_handles();

        on<Configuration>("NetworkForwarder.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            // Update which types we will be forwarding
            for (const auto& target_config : cfg["targets"]) {

                // Extract the target we are sending to
                auto target = target_config.first.as<std::string>();

                for (const auto& setting : target_config.second) {
                    // Get the name of the type
                    auto name = setting.first.as<std::string>();

                    double period = std::numeric_limits<double>::infinity();
                    bool enabled  = false;

                    // First we try reading this field as a double, if it is "true" or "false" it will throw an
                    // exception
                    try {
                        auto fps = setting.second.as<Expression>();
                        enabled  = fps != 0.0;
                        period   = enabled ? 1.0 / fps : std::numeric_limits<double>::infinity();
                    }
                    // If this happens we assume it's a boolean instead
                    catch (const YAML::TypedBadConversion<double>&) {
                        enabled = setting.second.as<bool>();
                        period  = enabled ? 0.0 : std::numeric_limits<double>::infinity();
                    }

                    // Message if we have enabled/disabled a particular message type
                    if (handles.find(name) != handles.end()) {
                        auto& handle = handles[name];
                        bool active  = handle->targets.count(target) != 0;

                        if (enabled && !active) {
                            log<NUClear::INFO>("Forwarding", name, "to", target);
                            handle->targets[target].period = period;
                        }
                        else if (!enabled && active) {
                            handle->targets.erase(target);
                            log<NUClear::INFO>("Stopped forwarding", name, "to", target);
                        }

                        // Enable if somebody wants this type
                        handle->reaction.enable(!handle->targets.empty());
                    }
                    else {
                        log<NUClear::WARN>("Network Forwarder does not know about the message type", name);
                    }
                }
            }
        });
    }

}  // namespace module::network
