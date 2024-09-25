#include "Rerun.hpp"

#include <cmath>
#include <string>

#include "extension/Configuration.hpp"

#include "message/eye/DataPoint.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/network.hpp"

namespace module::network {

    using extension::Configuration;
    using message::eye::DataPoint;
    using utility::nusight::graph;

    Rerun::Rerun(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Rerun.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            std::string recording_name = cfg["recording_name"].as<std::string>();
            hostname                   = utility::support::get_hostname();

            recording = std::make_unique<rerun::RecordingStream>(recording_name);
            // Connect to local host with default port.
            auto result = recording->connect();
            if (result.is_err()) {
                // Handle error.
                log<NUClear::ERROR>("Failed to connect to Rerun");
            }

            bool forward_datapoints = cfg["forward_datapoints"].as<bool>();
            forwarder_reaction.enable(forward_datapoints);

            if (forward_datapoints) {
                log<NUClear::INFO>("DataPoint forwarding setup for Rerun. Recording name:", recording_name);
            }
            else {
                log<NUClear::WARN>("DataPoint forwarding to Rerun disabled in Rerun.yaml config file");
            }
        });

        // Forward any DataPoint messages we get to Rerun
        forwarder_reaction = on<Trigger<DataPoint>>().then([this](const DataPoint& datapoint) {
            std::string entity_path = hostname + "/" + datapoint.label;

            if (datapoint.value.size() == 1) {
                recording->log(entity_path, rerun::Scalar(datapoint.value[0]));
            }
            else if (datapoint.value.size() < 5) {
                std::array<std::string, 4> keys{"x", "y", "z", "w"};
                for (size_t i = 0; i < std::min(datapoint.value.size(), size_t(3)); ++i) {
                    recording->log(entity_path + "/" + keys[i], rerun::Scalar(datapoint.value[i]));
                }
            }
            else {
                for (size_t i = 0; i < datapoint.value.size(); ++i) {
                    recording->log(entity_path + "/s" + std::to_string(i), rerun::Scalar(datapoint.value[i]));
                }
            }
        });
    }

}  // namespace module::network
