#ifndef MODULE_TOOLS_MOCAPODOMETRYBENCHMARK_HPP
#define MODULE_TOOLS_MOCAPODOMETRYBENCHMARK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

namespace module::tools {

    class MocapOdometryBenchmark : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief The rigid body id of the robot in motive software
            uint32_t robot_rigid_body_id = 0;
        } cfg;

        double total_error_x     = 0.0;
        double total_error_y     = 0.0;
        double total_error_z     = 0.0;
        double total_error_roll  = 0.0;
        double total_error_pitch = 0.0;
        double total_error_yaw   = 0.0;

        int count = 0;

        /// @brief The initialised ground truth Hfw, anchoring the mocap field frame to the robot's odometry world frame
        Eigen::Isometry3d ground_truth_Hfw = Eigen::Isometry3d::Identity();

        /// @brief Whether the ground truth Hfw has been initialised
        bool ground_truth_initialised = false;

    public:
        /// @brief Called by the powerplant to build and setup the MocapOdometryBenchmark reactor.
        explicit MocapOdometryBenchmark(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_MOCAPODOMETRYBENCHMARK_HPP
