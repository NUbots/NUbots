#include "PlotJuggler.hpp"

#include <cmath>
#include <json.hpp>
#include <string>

#include "extension/Configuration.hpp"

#include "message/eye/DataPoint.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/hostname.hpp"

namespace module::network {

    using extension::Configuration;
    using message::eye::DataPoint;
    using utility::nusight::graph;

    PlotJuggler::PlotJuggler(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {
        start_time_ms = toMillisecondsSinceEpoch(NUClear::clock::now());

        on<Configuration>("PlotJuggler.yaml").then([this](const Configuration& cfg) {
            log_level = cfg["log_level"].as<NUClear::LogLevel>();

            send_address = cfg["udp_server"]["ip_address"].as<std::string>();
            send_port    = cfg["udp_server"]["port"].as<int>();
            hostname     = utility::support::getHostname();

            bool forward_datapoints = cfg["forward_datapoints"].as<bool>();
            forwarder_reaction.enable(forward_datapoints);

            if (forward_datapoints) {
                log<NUClear::INFO>("DataPoint forwarding setup for PlotJuggler. Sending to UDP server at",
                                   send_address,
                                   "port",
                                   send_port);
            }
            else {
                log<NUClear::WARN>("DataPoint forwarding via UDP disabled in PlotJuggler.yaml config file");
            }

            bool send_debug_waves = cfg["send_debug_waves"].as<bool>();
            debug_waves_reaction.enable(send_debug_waves);

            if (send_debug_waves) {
                log<NUClear::INFO>("Debug waves enabled");
            }
        });

        // Forward any DataPoint messages we get to PlotJuggler via UDP
        forwarder_reaction = on<Trigger<DataPoint>>().then([this](const DataPoint& datapoint) {
            nlohmann::json json;

            // Set the timestamp in seconds relative to the start time
            json["timestamp"] = (toMillisecondsSinceEpoch(datapoint.timestamp) - start_time_ms) * 1e-3;

            // If there's only one value in the DataPoint, label it with the DataPoint's label without any nesting
            if (datapoint.value.size() == 1) {
                json[hostname][datapoint.label] = datapoint.value[0];
            }
            // If there're less than 5 values in the DataPoint, label them x, y, z, w respectively
            // and nest them in an JSON object labelled with the DataPoint's label
            else if (datapoint.value.size() < 5) {
                std::array<std::string, 4> keys{"x", "y", "z", "w"};

                json[hostname][datapoint.label] = {};

                for (uint i = 0; i < datapoint.value.size(); i++) {
                    json[hostname][datapoint.label][keys[i]] = datapoint.value[i];
                }
            }
            // Otherwise (with 5 or more values), label them s0, s1, s2, s3, etc
            // and nest them in a JSON object labelled with the DataPoint's label
            else {
                json[hostname][datapoint.label] = {};

                for (uint i = 0; i < datapoint.value.size(); i++) {
                    json[hostname][datapoint.label][fmt::format("s{}", i)] = datapoint.value[i];
                }
            }

            // Generate the JSON
            std::string json_string = json.dump();

            // Send it off to PlotJuggler
            auto packet = std::make_unique<std::string>(json_string);
            emit<Scope::UDP>(packet, send_address, send_port);
        });

        // Sends debug waves for testing the PlotJuggler connection
        debug_waves_reaction = on<Every<16, std::chrono::milliseconds>>().then([this] {
            long timestamp_ms        = toMillisecondsSinceEpoch(NUClear::clock::now()) - start_time_ms;
            double timestamp_seconds = timestamp_ms * 1e-3;

            emit(graph("Debug/Waves/sin", std::sin(timestamp_seconds)));
            emit(graph("Debug/Waves/cos",
                       std::cos(timestamp_seconds),
                       std::cos(timestamp_seconds) + 1,
                       std::cos(timestamp_seconds) + 2,
                       std::cos(timestamp_seconds) + 3));
            emit(graph("Debug/Waves/tan",
                       std::tan(timestamp_seconds),
                       std::tan(timestamp_seconds) + 1,
                       std::tan(timestamp_seconds) + 2,
                       std::tan(timestamp_seconds) + 3,
                       std::tan(timestamp_seconds) + 4));
        });
    }

}  // namespace module::network
