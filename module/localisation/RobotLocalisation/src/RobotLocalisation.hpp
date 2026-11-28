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

#include "message/input/Image.hpp"
#include "message/localisation/Field.hpp"
#include "message/purpose/Purpose.hpp"
#include "message/support/FieldDescription.hpp"

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
                    Eigen::Vector2d position        = Eigen::Vector2d::Zero();
                    Eigen::Vector2d velocity        = Eigen::Vector2d::Zero();
                    Eigen::Vector2d vision_velocity = Eigen::Vector2d::Zero();
                } initial_covariance{};
            } ukf{};

            /// @brief Parameters of the per-detection vision measurement covariance
            struct VisionMeasurement {
                double range_base        = 0.0;
                double range_quad        = 0.0;
                double bearing_per_metre = 0.0;
                double pose_std          = 0.0;
            } measurement{};

            /// @brief Squared Mahalanobis distance above which a detection cannot update a track
            double gate_chi2 = 0.0;
            /// @brief Seconds without a vision update after which a non-teammate track is removed
            double max_track_age = 0.0;
            /// @brief Tracks closer than this (m) to an older track are removed as duplicates
            double merge_distance = 0.0;
            /// @brief Maximum number of non-teammate tracks (0 = unlimited)
            int max_robot_tracks = 0;
            /// @brief The maximum number of times a teammate can be missed consecutively before it is removed
            int max_missed_count = 0;
            /// @brief How long (seconds) without a comms message before a teammate is removed. Teammates are
            /// pruned on comms staleness rather than the vision-driven missed_count, since they are frequently
            /// outside camera view but still broadcasting - see prediction() and maintenance().
            double teammate_timeout = 0.0;
            /// @brief The maximum distance a robot can be outside the field before it is ignored
            double max_distance_from_field = 0.0;
            /// @brief The maximum cost for a localisation to be considered valid
            double max_localisation_cost = 0.0;

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
            /// @brief Time of the last vision update (or creation)
            NUClear::clock::time_point last_vision_time = NUClear::clock::now();
            /// @brief The number of vision updates applied to this robot
            unsigned long vision_updates = 0;
            /// @brief A unique identifier for the robot
            unsigned long id;

            /// @brief The unique identifier of the robot if it is a teammate
            /// If it is not a teammate, this will be 0
            bool teammate                     = false;
            message::purpose::Purpose purpose = message::purpose::Purpose();
            /// @brief The last time a comms message was received for this teammate. Only meaningful when
            /// teammate is true.
            NUClear::clock::time_point last_comms_time = NUClear::clock::now();

            /// @brief Constructor that sets the state for the UKF
            /// @param initial_rRWw The initial position of the robot in world coordinates
            /// @param cfg_ukf The UKF configuration to use for the robot
            /// @param next_id The unique id to assign to this new robot
            /// @param p An optional purpose message that can be used to associate the robot with a teammate
            /// @param vision_R If given, the robot is created from a vision detection with this position covariance
            TrackedRobot(const Eigen::Vector3d& initial_rRWw,
                         const Config::UKF& cfg_ukf,
                         const unsigned long next_id,
                         const std::unique_ptr<message::purpose::Purpose>& p = nullptr,
                         const Eigen::Matrix2d* vision_R                     = nullptr)
                : id(next_id) {
                NUClear::log<NUClear::LogLevel::DEBUG>("Making robot with id: ", id);
                RobotModel<double>::StateVec initial_state;
                initial_state.rRWw = initial_rRWw.head<2>();

                RobotModel<double>::StateVec initial_covariance;
                initial_covariance.rRWw = cfg_ukf.initial_covariance.position;
                initial_covariance.vRw  = cfg_ukf.initial_covariance.velocity;
                Eigen::Matrix4d P0      = initial_covariance.asDiagonal();
                if (vision_R != nullptr) {
                    P0.setZero();
                    P0.topLeftCorner<2, 2>()     = *vision_R;
                    P0.bottomRightCorner<2, 2>() = cfg_ukf.initial_covariance.vision_velocity.asDiagonal();
                }
                ukf.set_state(initial_state, P0);

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

        /// @brief Associate vision detections with the tracked robots, gated by Mahalanobis distance and assigned
        /// one-to-one, lowest distance first. Unassociated detections create new tracks.
        /// @param robots_rRWw The detections in world coordinates
        /// @param covariances The measurement covariance of each detection
        void vision_association(const std::vector<Eigen::Vector3d>& robots_rRWw,
                                const std::vector<Eigen::Matrix2d>& covariances);

        /// @brief The covariance of a detection, from its range from the camera
        /// @param rRWw The detection in world coordinates
        /// @param rCWw The camera position in world coordinates
        Eigen::Matrix2d vision_covariance(const Eigen::Vector3d& rRWw, const Eigen::Vector3d& rCWw) const;

        /// @brief Run maintenance on the tracked robots
        /// This will remove any viewable robots that have been missed too many times or are too close to another robot
        /// @param image The most recent camera image, used to determine if a robot is within the camera's view
        /// @param field The field localisation, used to determine location of the tracked robot on field
        /// @param field_desc Field description, used to get the length and width of the field
        void maintenance(const message::input::Image& image,
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
