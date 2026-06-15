/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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
#ifndef MODULE_TOOLS_WALKDATACOLLECTOR_HPP
#define MODULE_TOOLS_WALKDATACOLLECTOR_HPP

#include <nuclear>
#include <random>
#include <tinyrobotics/inversekinematics.hpp>
#include <tinyrobotics/kinematics.hpp>
#include <tinyrobotics/parser.hpp>

#include "message/actuation/KinematicsModel.hpp"

#include "utility/skill/WalkGenerator.hpp"

namespace module::tools {

    class WalkDataCollector : public NUClear::Reactor {
    public:
        /// @brief Called by the powerplant to build and setup the WalkDataCollector reactor.
        explicit WalkDataCollector(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// @brief Number of actuatable joints in the NUgus robot (same as Kinematics module)
        static const int n_joints = 20;

        /// @brief Number of leg joints per leg
        static const int n_leg_joints = 6;

        /// @brief Total number of leg joints (both legs)
        static const int n_total_leg_joints = 2 * n_leg_joints;

        /// @brief Number of action history frames to include in observations
        static const int n_history_frames = 3;

        /// @brief Observation vector dimension:
        ///   3 (velocity cmd) + 2 (phase clock) + 1 (phase indicator) + 4 (engine state one-hot)
        ///   + n_history_frames * n_total_leg_joints (action history)
        static const int obs_dim = 3 + 2 + 1 + 4 + n_history_frames * n_total_leg_joints;

        /// @brief Target vector dimension: 12 leg joint angles
        static const int target_dim = n_total_leg_joints;

        /// @brief Total floats per sample (observation + target)
        static const int sample_dim = obs_dim + target_dim;

        /// @brief Stores configuration values
        struct Config {
            /// @brief Output directory for binary data files
            std::string output_directory = "walk_data";

            /// @brief Path to the URDF file
            std::string urdf_path = "";

            /// @brief IK tolerance
            double ik_tolerance = 1e-5;
            /// @brief IK max iterations
            int ik_max_iterations = 500;
            /// @brief IK method string
            std::string ik_method = "LEVENBERG_MARQUARDT";

            /// @brief Link names in tinyrobotics model
            std::string torso_name = "torso";
            std::string left_foot_name = "left_foot_base";
            std::string right_foot_name = "right_foot_base";

            /// @brief Walk engine parameters
            utility::skill::WalkGenerator<double>::WalkParameters walk_params{};

            /// @brief Velocity acceleration limits for smoothing
            Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

            /// @brief Number of episodes to generate
            int num_episodes = 10000;
            /// @brief Timesteps per episode
            int episode_length = 1000;
            /// @brief Update frequency in Hz
            int update_frequency = 100;
            /// @brief Random seed
            int seed = 42;
            /// @brief Velocity change interval [min, max] in seconds
            Eigen::Vector2d velocity_change_interval = Eigen::Vector2d(1.0, 3.0);

            /// @brief Velocity sampling ranges [min, max]
            Eigen::Vector2d vx_range = Eigen::Vector2d(-0.3, 0.5);
            Eigen::Vector2d vy_range = Eigen::Vector2d(-0.2, 0.2);
            Eigen::Vector2d vtheta_range = Eigen::Vector2d(-0.4, 0.4);
        } cfg;

        /// @brief tinyrobotics model for left leg IK
        tinyrobotics::Model<double, n_joints> nugus_model_left;

        /// @brief tinyrobotics model for right leg IK
        tinyrobotics::Model<double, n_joints> nugus_model_right;

        /// @brief tinyrobotics inverse kinematics options
        tinyrobotics::InverseKinematicsOptions<double, n_joints> ik_options;

        /// @brief Walk generator instance
        utility::skill::WalkGenerator<double> walk_generator;

        /// @brief KinematicsModel for analytical IK
        message::actuation::KinematicsModel kinematics_model;

        /// @brief Converts a string to a tinyrobotics InverseKinematicsMethod
        static tinyrobotics::InverseKinematicsMethod ik_string_to_method(const std::string& method_string);

        /**
         * @brief Run the full IK pipeline (analytical + numerical) for one leg.
         * @details Replicates the exact pipeline from module/actuation/Kinematics.
         * @param Htf Desired foot pose in torso frame
         * @param limb Which leg (LEFT_LEG or RIGHT_LEG)
         * @return Vector of 6 joint angles for the specified leg
         */
        std::array<double, n_leg_joints> compute_leg_ik(const Eigen::Isometry3d& Htf,
                                                        const utility::input::LimbID& limb);

        /**
         * @brief Sample a random velocity command.
         * @param rng Random number generator
         * @return Random velocity (vx, vy, vtheta)
         */
        Eigen::Vector3d sample_velocity(std::mt19937& rng);

        /**
         * @brief Run data collection for all episodes and write binary output files.
         */
        void collect_data();
    };

}  // namespace module::tools

#endif  // MODULE_TOOLS_WALKDATACOLLECTOR_HPP
