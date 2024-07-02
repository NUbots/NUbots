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

#include <Eigen/Core>
#include <nuclear>

#include "FieldModel.hpp"

#include "message/eye/DataPoint.hpp"
#include "message/localisation/Field.hpp"
#include "message/platform/RawSensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldLines.hpp"

#include "utility/localisation/FieldLineOccupanyMap.hpp"
#include "utility/localisation/OccupancyMap.hpp"
#include "utility/math/filter/ParticleFilter.hpp"
#include "utility/math/stats/multivariate.hpp"
#include "utility/nusight/NUhelpers.hpp"
#include "utility/support/yaml_expression.hpp"


namespace module::localisation {

    using message::platform::RawSensors;
    using message::support::FieldDescription;

    struct StartingSide {
        enum Value { UNKNOWN = 0, LEFT = 1, RIGHT = 2, EITHER = 3, CUSTOM = 4 };
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
                        else if (str == "CUSTOM") { value = Value::CUSTOM; }
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
                case Value::CUSTOM: return "CUSTOM";
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

            /// @brief Uncertainty in the process model (adds noise to the particles}
            Eigen::Matrix3d process_noise = Eigen::Matrix3d::Zero();

            /// @brief Initial state (x,y,theta) of the robot, saved for resetting
            Eigen::Vector3d initial_state{};

            /// @brief Initial covariance matrix of the robot's state, saved for resetting
            Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity();

            /// @brief Initial hypothesis of the robot's state (x,y,theta) and covariance, saved for resetting
            std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> initial_hypotheses{};

            /// @brief Bool to enable/disable saving the generated map as a csv file
            bool save_map = false;

            /// @brief Minimum number of field line points for a measurement update
            size_t min_observations = 0;

            /// @brief Start time delay for the particle filter
            double start_time_delay = 0.0;

            /// @brief Bool to enable/disable using ground truth for localisation
            bool use_ground_truth_localisation;

            /// @brief Starting side of the field (LEFT, RIGHT, EITHER, or CUSTOM)
            StartingSide starting_side = StartingSide::UNKNOWN;
            /// @brief Struct to store buzzer fields
            struct {
                /// @brief Container for the buzzer frequency when localisation is reset
                float localisation_reset_frequency = 0.0;
                /// @brief Container for the buzzer duration (milliseconds)
                int duration = 0;
            } buzzer;
        } cfg;

        /// @brief Last time filter was updated
        NUClear::clock::time_point last_time_update_time;

        /// @brief Particle filter class
        utility::math::filter::ParticleFilter<double, FieldModel> filter;

        /// @brief Field line distance map (encodes the minimum distance to a field line)
        utility::localisation::OccupancyMap<double> fieldline_distance_map;

        /// @brief Time at startup
        NUClear::clock::time_point startup_time;

    public:
        /// @brief Called by the powerplant to build and setup the FieldLocalisation reactor.
        explicit FieldLocalisation(std::unique_ptr<NUClear::Environment> environment);

        /**
         * @brief Compute Hfw, homogenous transformation from world {w} to field {f} space from state vector (x,y,theta)
         *
         * @param state The state vector (x,y,theta)
         * @return Hfw, the homogenous transformation matrix from world {w} to field {f} space
         */
        Eigen::Isometry3d compute_Hfw(const Eigen::Vector3d& particle);

        /**
         * @brief Find error between computed Hfw and ground truth if available
         *
         * @param Hfw Computed Hfw to be compared against ground truth
         * @param raw_sensors The raw sensor data
         */
        void debug_field_localisation(Eigen::Isometry3d Hfw, const RawSensors& raw_sensors);

        /**
         * @brief Transform a field line point from world {w} to position in the distance map {m}
         *
         * @param particle The state of the particle (x,y,theta)
         * @param rPWw The field point (x, y) in world space {w} [m]
         * @return Eigen::Vector2i
         */
        Eigen::Vector2i position_in_map(const Eigen::Vector3d& particle, const Eigen::Vector3d& rPWw);

        /**
         * @brief Calculate the weight of a particle given a set of observations
         *
         * @param particle The state of the particle (x,y,theta)
         * @param observations The observations (x, y) in the robot's coordinate frame [m]
         * @return Weight of the particle
         */
        double calculate_weight(const Eigen::Vector3d& particle, const std::vector<Eigen::Vector3d>& observations);

        /**
         * @brief Setup field line distance map
         *
         * @param fd The field dimensions
         */
        void setup_fieldline_distance_map(const FieldDescription& fd);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_FIELDLOCALISATION_HPP
