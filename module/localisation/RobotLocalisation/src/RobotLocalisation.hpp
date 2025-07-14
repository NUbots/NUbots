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
#ifndef MODULE_LOCALISATION_ROBOTLOCALISATION_HPP
#define MODULE_LOCALISATION_ROBOTLOCALISATION_HPP

#include <nuclear>
#include <vector>

#include "RobotModel.hpp"

#include "message/localisation/Field.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/GreenHorizon.hpp"

#include "utility/math/filter/UKF.hpp"

namespace module::localisation {
    class RobotLocalisation : public NUClear::Reactor {
    private:
        uint player_id = 1;

        struct Config {
            /// @brief UKF config
            struct UKF {
                struct Noise {
                    struct Measurement {
                        Eigen::Matrix2d position = Eigen::Matrix2d::Zero();
                    } measurement{};
                    struct Process {
                        Eigen::Vector2d position = Eigen::Vector2d::Zero();
                        Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                    } process{};
                } noise{};
                struct InitialCovariance {
                    Eigen::Vector2d position = Eigen::Vector2d::Zero();
                    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
                } initial_covariance{};
            } ukf{};

            /// @brief The maximum distance a measurement or other robot can be from another robot to be associated
            double association_distance = 0.0;
            /// @brief The maximum number of times a robot can be missed consecutively before it is removed
            int max_missed_count = 0;
            /// @brief The maximum distance a robot can be outside the field before it is ignored
            double max_distance_from_field = 0.0;
            /// @brief The maximum cost for a localisation to be considered valid
            double max_localisation_cost = 0.0;
            /// @brief Whether to use ground truth robot data from Webots when available
            bool use_ground_truth = false;

            bool force_playing = false;  // Temp: If we are force-playing, we need to default to our using RED as our team colour

        } cfg;

        struct TrackedRobot {
            /// @brief Time of the last time update
            NUClear::clock::time_point last_time_update = NUClear::clock::now();
            /// @brief Unscented Kalman Filter for this robot
            utility::math::filter::UKF<double, RobotModel> ukf{};
            /// @brief Whether the robot was and should have been seen in the last vision update
            bool seen = true;
            /// @brief The number of times the robot has been undetected in a row
            long missed_count = 0;
            /// @brief A unique identifier for the robot
            unsigned long id;

            /// @brief The unique identifier of the robot if it is a teammate
            /// If it is not a teammate, this will be 0
            bool teammate                     = false;
            message::purpose::Purpose purpose = message::purpose::Purpose();

            /// @brief Constructor that sets the state for the UKF
            /// @param initial_rRWw The initial position of the robot in world coordinates
            /// @param cfg_ukf The UKF configuration to use for the robot
            /// @param next_id The unique id to assign to this new robot
            /// @param p An optional purpose message that can be used to associate the robot with a teammate
            TrackedRobot(const Eigen::Vector3d& initial_rRWw,
                         const Config::UKF& cfg_ukf,
                         const unsigned long next_id,
                         const std::unique_ptr<message::purpose::Purpose>& p = nullptr)
                : id(next_id) {
                NUClear::log<NUClear::LogLevel::DEBUG>("Making robot with id: ", id);
                RobotModel<double>::StateVec initial_state;
                initial_state.rRWw = initial_rRWw.head<2>();

                RobotModel<double>::StateVec initial_covariance;
                initial_covariance.rRWw = cfg_ukf.initial_covariance.position;
                initial_covariance.vRw  = cfg_ukf.initial_covariance.velocity;
                ukf.set_state(initial_state, initial_covariance.asDiagonal());

                RobotModel<double>::StateVec process_noise;
                process_noise.rRWw      = cfg_ukf.noise.process.position;
                process_noise.vRw       = cfg_ukf.noise.process.velocity;
                ukf.model.process_noise = process_noise;

                // If a purpose is provided, set the teammate flag and purpose
                if (p) {
                    teammate = true;
                    purpose  = *p;
                }
            }

            // Get the robot's position in world space
            Eigen::Vector2d get_rRWw() const {
                return RobotModel<double>::StateVec(ukf.get_state()).rRWw;
            };
        };

        /// @brief List of tracked robots
        std::vector<TrackedRobot> tracked_robots{};

        /// @brief The next id to assign to a robot
        /// This variable will increase by one each time a new robot is added
        /// As it is unbounded, an unsigned long is used to store it
        unsigned long next_id = 0;

        /// @brief How many times per second to run the maintenance step
        static constexpr int UPDATE_RATE = 15;

        /// @brief Run Kalman filter prediction step for all tracked robots
        void prediction();

        /// @brief Associate the given robot measurements with the tracked robots
        /// Creates a new tracked robot if the measurement is not associated with an existing robot
        /// @param robots_rRWw The new robot measurements in world coordinates
        /// @param purpose An optional purpose message that can be used to associate the robot with teammate
        void data_association(const std::vector<Eigen::Vector3d>& robots_rRWw,
                              const std::unique_ptr<message::purpose::Purpose>& purpose = nullptr);

        /// @brief Run maintenance on the tracked robots
        /// This will remove any viewable robots that have been missed too many times or are too close to another robot
        /// @param horizon The green horizon from the vision system, to determine if a robot is in view
        /// @param field The field localisation, used to determine location of the tracked robot on field
        /// @param field_desc Field description, used to get the length and width of the field
        void maintenance(const message::vision::GreenHorizon& horizon,
                         const message::localisation::Field& field,
                         const message::support::FieldDescription& field_desc);

        /// @brief Print out the current state of the tracked robots
        void debug_info() const;

    public:
        /// @brief Called by the powerplant to build and setup the RobotLocalisation reactor.
        explicit RobotLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_ROBOTLOCALISATION_HPP
