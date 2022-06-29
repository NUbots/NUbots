#include "PlotJuggler.hpp"

#include <cmath>
#include <string>

#include "json.hpp"

#include "extension/Configuration.hpp"

#include "message/support/nusight/DataPoint.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/hostname.hpp"

namespace module::network {

    using extension::Configuration;
    using message::support::nusight::DataPoint;
    using utility::nusight::graph;

    PlotJuggler::PlotJuggler(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        start_time_ms = toMillisecondsSinceEpoch(NUClear::clock::now());

        on<Configuration>("PlotJuggler.yaml").then([this](const Configuration& cfg) {
            // TODO: setting this hangs the process and never returns, find a fix.
            // log_level = cfg["log_level"].as<NUClear::LogLevel>();

            forward_datapoints = cfg["forward_datapoints"].as<bool>();

            if (forward_datapoints) {
                send_address = cfg["udp_server"]["ip_address"].as<std::string>();
                send_port    = cfg["udp_server"]["port"].as<int>();
                hostname     = utility::support::getHostname();

                log<NUClear::INFO>("DataPoint forwarding setup for PlotJuggler. Sending to UDP server at",
                                   send_address,
                                   "port",
                                   send_port);
            }
            else {
                log<NUClear::WARN>("DataPoint forwarding via UDP disabled in PlotJuggler.yaml config file");
            }
        });

        // Forward any DataPoint messages we get to PlotJuggler via UDP
        on<Trigger<DataPoint>>().then([this](const DataPoint& datapoint) {
            // Abort early if we're not forwarding DataPoint messages
            if (!forward_datapoints) {
                return;
            }

            // Get our relative timestamp in seconds
            long timestamp_ms        = toMillisecondsSinceEpoch(datapoint.timestamp) - start_time_ms;
            double timestamp_seconds = timestamp_ms * 1e-3;

            nlohmann::json json;
            json["timestamp"] = timestamp_seconds;

            // If there is only one value in the DataPoint, label it with the DataPoint's label without any nesting
            if (datapoint.value.size() == 1) {
                json[hostname][datapoint.label] = datapoint.value[0];
            }
            // If there are less than 5 values in the DataPoint, label them x, y, z, w respectively
            // and nest them in an JSON object labelled with the DataPoint's label
            else if (datapoint.value.size() < 5) {
                std::array<std::string, 4> keys{"x", "y", "z", "w"};

                json[hostname][datapoint.label] = {};

                for (uint i = 0; i < datapoint.value.size(); i++) {
                    json[hostname][datapoint.label][keys[i]] = datapoint.value[i];
                }
            }
            // Otherwise (with 5 or more values), label them s0, s1, s2, s3, etc
            // and nest them in an JSON object labelled with the DataPoint's label
            else {
                json[hostname][datapoint.label] = {};

                for (uint i = 0; i < datapoint.value.size(); i++) {
                    json[hostname][datapoint.label][fmt::format("s{}", i)] = datapoint.value[i];
                }
            }

            // Add the data to the template
            std::string json_string = json.dump();
            // log<NUClear::INFO>("Sending JSON packet", json_string);

            // Send it off to PlotJuggler
            auto packet = std::make_unique<std::string>(json_string);
            emit<Scope::UDP>(packet, send_address, send_port);
        });

        // Test by sending sin and cos waves
        on<Every<16, std::chrono::milliseconds>>().then([this] {
            long timestamp_ms        = toMillisecondsSinceEpoch(NUClear::clock::now()) - start_time_ms;
            double timestamp_seconds = timestamp_ms * 1e-3;

            emit(graph("Sensor/Foot Down/Left", std::sin(timestamp_seconds)));
            emit(graph("Sensor/Foot Down/Right",
                       std::cos(timestamp_seconds),
                       std::cos(timestamp_seconds),
                       std::cos(timestamp_seconds),
                       std::cos(timestamp_seconds)));
            emit(graph("Sensor/Foot Down/Center",
                       std::cos(timestamp_seconds),
                       std::cos(timestamp_seconds),
                       std::cos(timestamp_seconds),
                       std::cos(timestamp_seconds),
                       std::cos(timestamp_seconds)));
        });
    }

}  // namespace module::network
