#include "NetworkForwarder.hpp"

#include <regex>
#include <string>

#include "extension/Configuration.hpp"

#include "utility/support/math_string.hpp"

namespace module::network {

    using extension::Configuration;

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

                    // Read in as a string and check for true or false
                    // Explicitly using the yaml supported variants, see https://yaml.org/type/bool.html
                    // If it is neither "true-ish" nor "false-ish", it must be "Expression-ish"
                    auto fps_str = setting.second.as<std::string>();
                    std::regex true_re("y|Y|yes|Yes|YES|true|True|TRUE|on|On|ON");
                    std::regex false_re("n|N|no|No|NO|false|False|FALSE|off|Off|OFF");
                    if (std::regex_match(fps_str, true_re)) {
                        enabled = true;
                        period  = 0.0;
                    }
                    else if (std::regex_match(fps_str, false_re)) {
                        enabled = false;
                        period  = std::numeric_limits<double>::infinity();
                    }
                    else {
                        auto fps = utility::support::parse_math_string<double>(fps_str);
                        enabled  = fps > 0.0;
                        period   = enabled ? 1.0 / fps : std::numeric_limits<double>::infinity();
                    }

                    // Message if we have enabled/disabled a particular message type
                    if (handles.contains(name)) {
                        auto& handle = handles[name];
                        bool active  = handle->targets.contains(target);

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
