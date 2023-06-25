#ifndef MODULE_TOOLS_CALIBRATEIMU_HPP
#define MODULE_TOOLS_CALIBRATEIMU_HPP

#include <Eigen/Core>
#include <nuclear>

namespace module::tools {

    class CalibrateIMU : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Expected accelerometer values for stationary IMU
            Eigen::Vector3d expected_accelerometer = Eigen::Vector3d::Zero();
        } cfg;

    public:
        /// @brief Called by the powerplant to build and setup the CalibrateIMU reactor.
        explicit CalibrateIMU(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_CALIBRATEIMU_HPP
