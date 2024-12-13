/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
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
#include "LocalisationBenchmark.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tinyrobotics/math.hpp>

#include "extension/Configuration.hpp"

#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"

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
    using message::localisation::Field;
    using message::localisation::ResetFieldLocalisation;
    using message::platform::RawSensors;

    using NUClear::message::CommandLineArguments;

    LocalisationBenchmark::LocalisationBenchmark(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("LocalisationBenchmark.yaml").then([this](const Configuration& cfg) {
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
                log<NUClear::ERROR>("Playback mode is invalid, stopping playback");
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

        on<Trigger<ResetFieldLocalisation>>().then([this]() {
            total_localisation_translation_error = 0.0;
            total_localisation_rotation_error    = 0.0;
        });

        on<Startup, With<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            // Set playback mode
            auto set_mode_request  = std::make_unique<SetModeRequest>();
            set_mode_request->mode = config.mode;
            emit<Scope::DIRECT>(set_mode_request);

            // Delay
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Load the files
            auto load_request      = std::make_unique<LoadRequest>();
            load_request->files    = std::vector<std::string>(std::next(args.begin()), args.end());
            load_request->messages = config.messages;
            emit<Scope::DIRECT>(std::move(load_request));

            // Delay
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Start playback
            emit<Scope::DIRECT>(std::make_unique<PlayRequest>());
        });


        on<Trigger<Field>, With<Sensors>, With<RawSensors>>().then(
            [this](const Field& field, const Sensors& sensors, const RawSensors& raw_sensors) {
                // Compute estimated Hft
                Eigen::Isometry3d Hft_est = field.Hfw * sensors.Htw.inverse();

                // Compute ground truth Hft
                Eigen::Isometry3d Hft_gt = Eigen::Isometry3d(raw_sensors.localisation_ground_truth.Hfw)
                                           * Eigen::Isometry3d(raw_sensors.odometry_ground_truth.Htw).inverse();

                // Compute localisation error
                auto localisation_error               = tinyrobotics::homogeneous_error(Hft_gt, Hft_est);
                double localisation_translation_error = localisation_error.head<3>().norm();
                total_localisation_translation_error += localisation_translation_error * localisation_translation_error;

                // Compute localisation rotation error
                double localisation_rotation_error = localisation_error.tail<3>().norm();
                total_localisation_rotation_error += localisation_rotation_error * localisation_rotation_error;

                // Accumulate squared errors for each individual DoF
                total_error_x += std::pow(localisation_error(0), 2);
                total_error_y += std::pow(localisation_error(1), 2);
                total_error_z += std::pow(localisation_error(2), 2);
                total_error_roll += std::pow(localisation_error(3), 2);
                total_error_pitch += std::pow(localisation_error(4), 2);
                total_error_yaw += std::pow(localisation_error(5), 2);

                // Increment counter
                count++;
            });

        on<Trigger<PlaybackFinished>>().then([this] {
            double localisation_rmse_translation = std::sqrt(total_localisation_translation_error / count);
            double localisation_rmse_rotation    = std::sqrt(total_localisation_rotation_error / count);

            // Calculate RMSE for each DoF
            double rmse_x     = std::sqrt(total_error_x / count);
            double rmse_y     = std::sqrt(total_error_y / count);
            double rmse_z     = std::sqrt(total_error_z / count);
            double rmse_roll  = std::sqrt(total_error_roll / count);
            double rmse_pitch = std::sqrt(total_error_pitch / count);
            double rmse_yaw   = std::sqrt(total_error_yaw / count);

            std::cout << "translation rmse error: " << localisation_rmse_translation << std::endl;
            std::cout << "rotation rmse error: " << localisation_rmse_rotation * 180.0 / M_PI << " degrees"
                      << std::endl;
            std::cout << "RMSE (x): " << rmse_x << std::endl;
            std::cout << "RMSE (y): " << rmse_y << std::endl;
            std::cout << "RMSE (z): " << rmse_z << std::endl;
            std::cout << "RMSE (roll): " << rmse_roll * 180.0 / M_PI << " degrees" << std::endl;
            std::cout << "RMSE (pitch): " << rmse_pitch * 180.0 / M_PI << " degrees" << std::endl;
            std::cout << "RMSE (yaw): " << rmse_yaw * 180.0 / M_PI << " degrees" << std::endl;

            // Kill program
            powerplant.shutdown();
        });
    }

}  // namespace module::tools
