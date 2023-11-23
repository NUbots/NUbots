/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#ifndef MODULE_LOCALISATION_FIELDLOCALISATION_HPP
#define MODULE_LOCALISATION_FIELDLOCALISATION_HPP

#include <nuclear>

#include "message/eye/DataPoint.hpp"
#include "message/localisation/Field.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldLines.hpp"

#include "utility/localisation/OccupancyMap.hpp"
#include "utility/math/stats/multivariate.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::localisation {

    // Particle struct
    struct Particle {
        Eigen::Vector3d state = Eigen::Vector3d::Zero();  // (x, y, theta) of world space in field space
        double weight         = 1.0;
    };


    struct StartingSide {
        enum Value { UNKNOWN = 0, LEFT = 1, RIGHT = 2, EITHER = 3 };
        Value value = Value::UNKNOWN;

        // Constructors
        StartingSide() = default;
        StartingSide(int const& v) : value(static_cast<Value>(v)) {}
        StartingSide(Value const& v) : value(v) {}
        StartingSide(std::string const& str) {
            // clang-format off
                        if      (str == "LEFT") { value = Value::LEFT; }
                        else if (str == "RIGHT") { value = Value::RIGHT; }
                        else if (str == "EITHER")  { value = Value::EITHER; }
                        else {
                            value = Value::UNKNOWN;
                            throw std::runtime_error("String " + str + " did not match any enum for StartingSide");
                        }
            // clang-format on
        }

        // Conversions
        [[nodiscard]] operator Value() const {
            return value;
        }
        [[nodiscard]] operator std::string() const {
            switch (value) {
                case Value::LEFT: return "LEFT";
                case Value::RIGHT: return "RIGHT";
                case Value::EITHER: return "EITHER";
                default: throw std::runtime_error("enum Method's value is corrupt, unknown value stored");
            }
        }
    };

    class FieldLocalisation : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Size of the grid cells in the occupancy grid [m]
            double grid_size = 0.0;
            /// @brief Number of particles to use in the particle filter
            int n_particles = 0;
            /// @brief Uncertainty in the process model
            Eigen::Matrix3d process_noise = Eigen::Matrix3d::Zero();
            /// @brief Uncertainty in the measurement model
            double measurement_noise = 0;
            /// @brief Maximum distance a field line can be from a particle to be considered an observation [m]
            double max_range = 0;
            /// @brief Initial state (x,y,theta) of the robot, saved for resetting
            std::vector<Eigen::Vector3d> initial_state{};
            /// @brief Initial covariance matrix of the robot's state, saved for resetting
            Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity();
            /// @brief Bool to enable/disable saving the generated map as a csv file
            bool save_map = false;
            /// @brief Minimum number of field line points for a measurement update
            size_t min_observations = 0;
            /// @brief Penalty factor for observations being outside map
            double outside_map_penalty_factor = 0.0;
            /// @brief Start time delay for the particle filter
            double start_time_delay = 0.0;
            /// @brief Starting side of the field (left, right, or either)
            StartingSide starting_side = StartingSide::UNKNOWN;
        } cfg;

        NUClear::clock::time_point last_time_update_time;

        /// @brief Occupancy grid map of the field lines
        OccupancyMap<double> fieldline_map;

        /// @brief State (x,y,theta) of the robot
        Eigen::Vector3d state = Eigen::Vector3d::Zero();

        /// @brief Covariance matrix of the robot's state
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

        /// @brief Status of if the robot is falling
        bool falling = false;

        /// @brief Particles used in the particle filter
        std::vector<Particle> particles{};

        /// @brief The time of startup
        NUClear::clock::time_point startup_time;

    public:
        /// @brief Called by the powerplant to build and setup the FieldLocalisation reactor.
        explicit FieldLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /// @brief Transform a point in the robot's coordinate frame into an index in the map
        /// @param particle The state of the particle (x,y,theta)
        /// @param rPWw The field point (x, y) in world space {w} [m]
        /// @return The observation location (x, y) in the map
        Eigen::Vector2i position_in_map(const Eigen::Vector3d particle, const Eigen::Vector3d rPWw);

        /// @brief Get the weight of a particle given a set of observations
        /// @param particle The state of the particle (x,y,theta)
        /// @param observations The observations (x, y) in the robot's coordinate frame [m]
        /// @return The weight of the particle
        double calculate_weight(const Eigen::Vector3d particle, const std::vector<Eigen::Vector3d>& observations);

        /// @brief Get the current mean (state) of the robot
        // @return The current mean (state) of the robot
        Eigen::Vector3d compute_mean();

        /// @brief Get the current covariance matrix of the robot's state
        // @return The current covariance matrix of the robot's state
        Eigen::Matrix3d compute_covariance();

        /// @brief Perform a time update on the particles
        /// @param walk_command The walk command (dx, dy, dtheta)
        /// @param dt The time since the last time update
        void time_update();

        /// @brief Resample the particles based on their weights
        /// @param particles Vector of particles to be resampled
        /// @return The resampled particles
        void resample();

        /// @brief Add some noise to the particles to compensate for the fact that we don't have a perfect model
        /// @param particles Vector of particles to be resampled
        /// @return The particles with noise added
        void add_noise();
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_FIELDLOCALISATION_HPP
