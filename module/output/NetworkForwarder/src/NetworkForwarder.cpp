#include "NetworkForwarder.hpp"

#include "extension/Configuration.hpp"

namespace module::output {

    using extension::Configuration;

    NetworkForwarder::NetworkForwarder(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        // Register the message forwarding
        register_handles();

        on<Configuration>("NetworkForwarder.yaml").then([this](const Configuration& cfg) {
            // clang-format off
            auto lvl = cfg["log_level"].as<std::string>();
            if (lvl == "TRACE") { this->log_level = NUClear::TRACE; }
            else if (lvl == "DEBUG") { this->log_level = NUClear::DEBUG; }
            else if (lvl == "INFO") { this->log_level = NUClear::INFO; }
            else if (lvl == "WARN") { this->log_level = NUClear::WARN; }
            else if (lvl == "ERROR") { this->log_level = NUClear::ERROR; }
            else if (lvl == "FATAL") { this->log_level = NUClear::FATAL; }
            // clang-format on

            config.target = cfg["target"].as<std::string>();

            // Update which types we will be forwarding
            for (const auto& setting : cfg["messages"].config) {
                // Get the name of the type
                auto name = setting.first.as<std::string>();

                double period = std::numeric_limits<double>::infinity();
                bool enabled  = false;

                // First we try reading this field as a double, if it is "true" or "false" it will throw an exception
                try {
                    auto fps = setting.second.as<double>();
                    period   = fps == 0.0 ? std::numeric_limits<double>::infinity() : 1.0 / fps;
                    enabled  = fps != 0.0;
                }
                // If this happens we assume it's a boolean instead
                catch (const YAML::TypedBadConversion<double>&) {
                period  = std::numeric_limits<double>::infinity();
                    enabled = setting.second.as<bool>();
                }

                // Message if we have enabled/disabled a particular message type
                if (handles.find(name) != handles.end()) {
                    auto& handle = handles[name];

                    handle->period = period;

                    if (enabled && !handle->reaction.enabled()) {
                        handle->reaction.enable();
                        log<NUClear::INFO>("Forwarding", name, "to", config.target);
                    }
                    else if (!enabled && handle->reaction.enabled()) {
                        handle->reaction.disable();
                        log<NUClear::INFO>("Stopped forwarding", name, "to", config.target);
                    }
                }
                else {
                    log<NUClear::WARN>("Network Forwarder does not know about the message type", name);
                }
            }
        });
    }

}  // namespace module::output
