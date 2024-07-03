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

#include "utility/math/filter/UKF.hpp"

namespace module::localisation {
    class RobotLocalisation : public NUClear::Reactor {
    private:
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
            const unsigned long id;

            /// @brief Constructor that sets the state for the UKF
            TrackedRobot(const Eigen::Vector3d& initial_rRWw, const Config::UKF& cfg_ukf, const unsigned long next_id)
                : id(next_id) {
                NUClear::log<NUClear::DEBUG>("Making robot with id: ", id);
                RobotModel<double>::StateVec initial_state = Eigen::Matrix<double, 4, 1>::Zero();
                initial_state.rRWw                         = initial_rRWw.head<2>();

                ukf.set_state(initial_state.getStateVec(),
                              RobotModel<double>::StateVec(cfg_ukf.initial_covariance.position).asDiagonal());
                ukf.model.process_noise = RobotModel<double>::StateVec(cfg_ukf.noise.process.position);
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

    public:
        /// @brief Called by the powerplant to build and setup the RobotLocalisation reactor.
        explicit RobotLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Tests if this robot measurement is associated with a tracked robot or if it is a new robot
        /// @param vision_robot The robot detection from the vision system
        void data_association(const Eigen::Vector3d& rRWw);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_ROBOTLOCALISATION_HPP
