#include "MocapOdometryBenchmark.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <tinyrobotics/math.hpp>

#include "extension/Configuration.hpp"

#include "message/input/MotionCapture.hpp"
#include "message/input/Sensors.hpp"

#include "utility/math/euler.hpp"

namespace module::tools {

    using extension::Configuration;
    using message::input::MotionCapture;
    using message::input::Sensors;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;

    MocapOdometryBenchmark::MocapOdometryBenchmark(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("MocapOdometryBenchmark.yaml").then([this](const Configuration& config) {
            // Use configuration here from file MocapOdometryBenchmark.yaml
            this->log_level         = config["log_level"].as<NUClear::LogLevel>();
            cfg.robot_rigid_body_id = config["robot_rigid_body_id"].as<uint32_t>();
        });

        on<Trigger<Sensors>, With<MotionCapture>>().then([this](const Sensors& sensors, const MotionCapture& mocap) {
            // Process mocap and sensor data here to compute odometry error

            // this bit's easy - get the estimated Htw from the sensors message
            Eigen::Isometry3d Htw_est = sensors.Htw;

            // this bit's harder - get the ground truth Htw from the mocap message
            auto rigid_body = std::find_if(mocap.rigid_bodies.begin(),
                                           mocap.rigid_bodies.end(),
                                           [this](const auto& rb) { return rb.id == cfg.robot_rigid_body_id; });
            if (rigid_body == mocap.rigid_bodies.end() || !rigid_body->tracking_valid) {
                return;
            }

            // Build Hft from the mocap rigid body (same coordinate correction as the Mocap module)
            Eigen::Isometry3d Hft = Eigen::Isometry3d::Identity();
            Hft.translation() =
                Eigen::Vector3d(-rigid_body->position.y(), rigid_body->position.x(), rigid_body->position.z());
            auto rpy     = mat_to_rpy_intrinsic(Eigen::Quaterniond(rigid_body->rotation(0),
                                                                   rigid_body->rotation(1),
                                                                   rigid_body->rotation(2),
                                                                   rigid_body->rotation(3))
                                                    .toRotationMatrix());
            Hft.linear() = rpy_intrinsic_to_mat(Eigen::Vector3d(rpy.y(), -(rpy.z() - M_PI), rpy.x()));

            // Anchor the mocap field frame to the robot's odometry world frame on the first valid reading, since
            // the two coordinate systems do not share an origin or heading
            if (!ground_truth_initialised) {
                ground_truth_Hfw.translation().head<2>() = Hft.translation().head<2>();
                ground_truth_Hfw.translation().z()       = 0;
                ground_truth_Hfw.linear() =
                    rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, mat_to_rpy_intrinsic(Hft.rotation()).z()));
                ground_truth_initialised = true;
            }

            Eigen::Isometry3d Htw_gt = Hft.inverse() * ground_truth_Hfw;

            // Compute odometry error
            Eigen::Matrix<double, 6, 1> odometry_error = tinyrobotics::homogeneous_error(Htw_gt, Htw_est);
            double odometry_translation_error          = odometry_error.head<3>().norm();
            double total_odometry_translation_error    = 0.0;
            total_odometry_translation_error += odometry_translation_error * odometry_translation_error;

            // Compute odometry rotation error, ignore yaw for now
            // TODO: Add yaw error
            double odometry_rotation_error       = odometry_error.tail<3>().head<2>().norm();
            double total_odometry_rotation_error = 0.0;
            total_odometry_rotation_error += odometry_rotation_error * odometry_rotation_error;

            // Accumulate squared errors for each individual DoF
            total_error_x += std::pow(odometry_error(0), 2);
            total_error_y += std::pow(odometry_error(1), 2);
            total_error_z += std::pow(odometry_error(2), 2);
            total_error_roll += std::pow(odometry_error(3), 2);
            total_error_pitch += std::pow(odometry_error(4), 2);
            total_error_yaw += std::pow(odometry_error(5), 2);

            count++;

            double odometry_rmse_translation = std::sqrt(total_odometry_translation_error / count);
            double odometry_rmse_rotation    = std::sqrt(total_odometry_rotation_error / count);

            // Calculate RMSE for each DoF
            double rmse_x     = std::sqrt(total_error_x / count);
            double rmse_y     = std::sqrt(total_error_y / count);
            double rmse_z     = std::sqrt(total_error_z / count);
            double rmse_roll  = std::sqrt(total_error_roll / count);
            double rmse_pitch = std::sqrt(total_error_pitch / count);
            double rmse_yaw   = std::sqrt(total_error_yaw / count);

            log<INFO>("Current odometry translation RMSE error: %f m", odometry_rmse_translation);
            log<INFO>("Current odometry rotation RMSE error: %f degrees", odometry_rmse_rotation * 180.0 / M_PI);
            log<INFO>("Current odometry RMSE (x): %f m", rmse_x);
            log<INFO>("Current odometry RMSE (y): %f m", rmse_y);
            log<INFO>("Current odometry RMSE (z): %f m", rmse_z);
            log<INFO>("Current odometry RMSE (roll): %f degrees", rmse_roll * 180.0 / M_PI);
            log<INFO>("Current odometry RMSE (pitch): %f degrees", rmse_pitch * 180.0 / M_PI);
            log<INFO>("Current odometry RMSE (yaw): %f degrees", rmse_yaw * 180.0 / M_PI);
        });
    }

}  // namespace module::tools
