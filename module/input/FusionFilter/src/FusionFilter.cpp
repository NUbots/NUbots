#include "FusionFilter.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "build.hpp"  // TODO(KipHamiltons): For NED definition. Remove once that's been sorted out
#include "filter/kalman.hpp"

#include "extension/Configuration.hpp"

namespace module::input {

    using extension::Configuration;
    using message::input::OrientationReading;
    using message::input::ResetFusionFilter;

    FusionFilter::FusionFilter(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("FusionFilter.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file FusionFilter.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
            // Initialise the filter
            filter::kalman::fInit_6DOF_GY_KALMAN(filter);
        });

        on<OrientationReading>().then([this](const OrientationReading& reading) {
            const Eigen::Vector3d accel = reading.acceleration;
            const Eigen::Vector3d gyro  = reading.gyro;
            // TODO(KipHamiltons): sort out coordinate system
            // Run the filter, feeding it the acceleration and gyro readings from the message
            // It outputs an orientation prediction, which is a rotation quaternion
            const Eigen::Quaternion<double> rotation_prediction =
                filter::kalman::fRun_6DOF_GY_KALMAN(filter, acceleration, gyro, NED);

            auto rotation_prediction       = std::make_unique<RotationPrediction>();
            rotation_prediction.prediction = rotation_prediction.toRotationMatrix();
            emit(std::move(rotation_prediction));
        });

        // Empty message indicating the filter should be reset
        on<ResetFusionFilter>().then([this](const ResetFusionFilter& /*msg*/) {
            // Reset the filter in place
            filter::kalman::fInit_6DOF_GY_KALMAN(filter);
        });
    }

}  // namespace module::input
