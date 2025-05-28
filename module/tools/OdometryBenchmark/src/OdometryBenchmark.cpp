/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "OdometryBenchmark.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyrobotics/math.hpp>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"

#include "utility/math/euler.hpp"

namespace module::tools {

    using extension::Configuration;

    using message::nbs::player::LoadRequest;
    using message::nbs::player::PauseRequest;
    using message::nbs::player::PlaybackFinished;
    using message::nbs::player::PlaybackState;
    using message::nbs::player::PlayRequest;
    using message::nbs::player::SetModeRequest;
    using message::nbs::player::PlaybackMode::FAST;
    using message::nbs::player::PlaybackMode::REALTIME;
    using message::nbs::player::PlaybackMode::SEQUENTIAL;

    using message::input::Sensors;

    using message::localisation::RobotPoseGroundTruth;

    using NUClear::message::CommandLineArguments;

    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;

    OdometryBenchmark::OdometryBenchmark(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("OdometryBenchmark.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file OdometryBenchmark.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();

            auto playback_mode = cfg["playback_mode"].as<std::string>();
            if (playback_mode == "FAST") {
                config.mode = FAST;
            }
            else if (playback_mode == "SEQUENTIAL") {
                config.mode = SEQUENTIAL;
            }
            else if (playback_mode == "REALTIME") {
                config.mode = REALTIME;
            }
            else {
                log<ERROR>("Playback mode is invalid, stopping playback");
                powerplant.shutdown();
            }

            // Update which types we will be playing
            for (const auto& setting : cfg["messages"]) {
                auto name    = setting.first.as<std::string>();
                bool enabled = setting.second.as<bool>();
                if (enabled) {
                    config.messages.push_back(name);
                }
            }
        });

        on<Startup, With<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            // Set playback mode
            auto set_mode_request  = std::make_unique<SetModeRequest>();
            set_mode_request->mode = config.mode;
            emit<Scope::INLINE>(set_mode_request);

            // Delay
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Load the files
            auto load_request      = std::make_unique<LoadRequest>();
            load_request->files    = std::vector<std::string>(std::next(args.begin()), args.end());
            load_request->messages = config.messages;
            emit<Scope::INLINE>(std::move(load_request));

            // Delay
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Start playback
            emit<Scope::INLINE>(std::make_unique<PlayRequest>());
        });

        on<Trigger<Sensors>, With<RobotPoseGroundTruth>>().then(
            [this](const Sensors& sensors, const RobotPoseGroundTruth& robot_pose_ground_truth) {
                Eigen::Isometry3d Hft = Eigen::Isometry3d(robot_pose_ground_truth.Hft);
                if (!ground_truth_initialised) {
                    // Initialise the ground truth Hfw
                    ground_truth_Hfw.translation().head<2>() = Hft.translation().head<2>();
                    ground_truth_Hfw.translation()[2]        = 0;
                    double yaw                               = mat_to_rpy_intrinsic(Hft.rotation()).z();
                    ground_truth_Hfw.linear()                = rpy_intrinsic_to_mat(Eigen::Vector3d(0, 0, yaw));
                    ground_truth_initialised                 = true;
                }


                // Compute estimated Htw
                Eigen::Isometry3d Htw_est = sensors.Htw;

                // Compute ground truth Htw
                Eigen::Isometry3d Htw_gt = ground_truth_Hfw.inverse() * Hft;

                // Compute odometry error
                Eigen::Matrix<double, 6, 1> odometry_error = tinyrobotics::homogeneous_error(Htw_gt, Htw_est);
                double odometry_translation_error          = odometry_error.head<3>().norm();
                total_odometry_translation_error += odometry_translation_error * odometry_translation_error;

                // Compute odometry rotation error, ignore yaw for now
                // TODO: Add yaw error
                double odometry_rotation_error = odometry_error.tail<3>().head<2>().norm();
                total_odometry_rotation_error += odometry_rotation_error * odometry_rotation_error;

                // Accumulate squared errors for each individual DoF
                total_error_x += std::pow(odometry_error(0), 2);
                total_error_y += std::pow(odometry_error(1), 2);
                total_error_z += std::pow(odometry_error(2), 2);
                total_error_roll += std::pow(odometry_error(3), 2);
                total_error_pitch += std::pow(odometry_error(4), 2);
                total_error_yaw += std::pow(odometry_error(5), 2);

                // Increment counter
                count++;
            });

        on<Trigger<PlaybackFinished>>().then([this] {
            // Check if we have any data before dividing by count
            if (count == 0) {
                log<ERROR>("No odometry data processed.");
                powerplant.shutdown();
                return;
            }

            double odometry_rmse_translation = std::sqrt(total_odometry_translation_error / count);
            double odometry_rmse_rotation    = std::sqrt(total_odometry_rotation_error / count);

            // Calculate RMSE for each DoF
            double rmse_x     = std::sqrt(total_error_x / count);
            double rmse_y     = std::sqrt(total_error_y / count);
            double rmse_z     = std::sqrt(total_error_z / count);
            double rmse_roll  = std::sqrt(total_error_roll / count);
            double rmse_pitch = std::sqrt(total_error_pitch / count);
            double rmse_yaw   = std::sqrt(total_error_yaw / count);

            std::cout << "Odometry translation RMSE error: " << odometry_rmse_translation << " m" << std::endl;
            std::cout << "Odometry rotation RMSE error: " << odometry_rmse_rotation * 180.0 / M_PI << " degrees"
                      << std::endl;
            std::cout << "Odometry RMSE (x): " << rmse_x << " m" << std::endl;
            std::cout << "Odometry RMSE (y): " << rmse_y << " m" << std::endl;
            std::cout << "Odometry RMSE (z): " << rmse_z << " m" << std::endl;
            std::cout << "Odometry RMSE (roll): " << rmse_roll * 180.0 / M_PI << " degrees" << std::endl;
            std::cout << "Odometry RMSE (pitch): " << rmse_pitch * 180.0 / M_PI << " degrees" << std::endl;
            std::cout << "Odometry RMSE (yaw): " << rmse_yaw * 180.0 / M_PI << " degrees" << std::endl;

            // Kill program
            powerplant.shutdown();
        });
    }

}  // namespace module::tools
