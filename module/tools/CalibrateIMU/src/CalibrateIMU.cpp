#include "CalibrateIMU.hpp"

#include "extension/Configuration.hpp"

#include "message/platform/RawSensors.hpp"

#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"

namespace module::tools {

    using extension::Configuration;

    using utility::nusight::graph;
    using utility::support::Expression;

    using message::platform::RawSensors;

    CalibrateIMU::CalibrateIMU(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("CalibrateIMU.yaml").then([this](const Configuration& config) {
            // Use configuration here from file CalibrateIMU.yaml
            this->log_level            = config["log_level"].as<NUClear::LogLevel>();
            cfg.expected_accelerometer = config["expected_accelerometer"].as<Expression>();
        });

        on<Last<5000, Trigger<RawSensors>>, Single>().then(
            [this](const std::list<std::shared_ptr<const RawSensors>>& raw_sensors) {
                // Compute the average accelerometer and gyroscope values
                Eigen::Vector3d accelerometer_average = Eigen::Vector3d::Zero();
                Eigen::Vector3d gyroscope_average     = Eigen::Vector3d::Zero();
                for (const auto& raw_sensor : raw_sensors) {
                    accelerometer_average += raw_sensor->accelerometer.cast<double>();
                    gyroscope_average += raw_sensor->gyroscope.cast<double>();
                }
                accelerometer_average /= raw_sensors.size();
                gyroscope_average /= raw_sensors.size();
                emit(graph("Accelerometer average (x,y,z)",
                           accelerometer_average.x(),
                           accelerometer_average.y(),
                           accelerometer_average.z()));
                emit(graph("Gyroscope average (x,y,z)",
                           gyroscope_average.x(),
                           gyroscope_average.y(),
                           gyroscope_average.z()));

                // Compute the bias as the difference between the measured and expected
                Eigen::Vector3d accelerometer_bias = accelerometer_average - cfg.expected_accelerometer;

                // Emit the bias
                emit(graph("Accelerometer bias (x,y,z)",
                           accelerometer_bias.x(),
                           accelerometer_bias.y(),
                           accelerometer_bias.z()));
                log<NUClear::INFO>("Accelerometer bias: {} {} {}",
                                   accelerometer_bias.x(),
                                   accelerometer_bias.y(),
                                   accelerometer_bias.z());
            });
    }

}  // namespace module::tools
