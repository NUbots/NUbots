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
#ifndef MODULES_LOCALISATION_ROBOTMODEL_HPP
#define MODULES_LOCALISATION_ROBOTMODEL_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "message/vision/FieldIntersections.hpp"

namespace module::localisation {

    /// @brief Landmark struct
    struct Landmark {
        /// @brief Landmark position in field space
        Eigen::Vector2d position{};

        /// @brief Landmark type
        message::vision::FieldIntersection::IntersectionType type =
            message::vision::FieldIntersection::IntersectionType::UNKNOWN;
    };


    template <typename Scalar>
    class FieldModel {
    public:
        /// @brief Size of the state vector (x, y, theta)
        static constexpr size_t size = 3;

        using StateVec = Eigen::Matrix<Scalar, size, 1>;
        using StateMat = Eigen::Matrix<Scalar, size, size>;

        enum Components : int {
            /// @brief World {w} x position from field {f}
            x = 0,

            /// @brief World {w} y position from field {f}
            y = 1,

            /// @brief Angle of world {w} from field {f}
            theta = 2
        };

        /// @brief Number of reset particles
        int n_rogues = 0;

        /// @brief Number of particles
        int n_particles = 100;

        /// @brief Range of reset particles
        Eigen::Matrix<Scalar, 3, 1> reset_range = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Diagonal for 3x3 noise matrix (which is diagonal)
        Eigen::Matrix<Scalar, 3, 1> process_noise_diagonal = Eigen::Matrix<Scalar, 3, 1>::Ones();

        FieldModel()
            : reset_range(Eigen::Matrix<Scalar, 3, 1>::Zero())
            , process_noise_diagonal(Eigen::Matrix<Scalar, 3, 1>::Ones()) {}

        StateVec time(const StateVec& state, double /*dt*/) {
            return state;
        }

        StateVec limit(const StateVec& state) {
            return state;
        }

        StateMat noise(const Scalar& dt) {
            return process_noise_diagonal.asDiagonal() * dt;
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, 1> predict(
            const StateVec& state,
            const message::vision::FieldIntersection& observed_intersection,
            const std::vector<Landmark>& landmarks,
            const double min_association_distance) {

            double min_distance = std::numeric_limits<double>::max();
            Eigen::Vector2d closest_landmark_position =
                Eigen::Vector2d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

            auto Hfw = Eigen::Translation<double, 2>(state.x(), state.y()) * Eigen::Rotation2D<double>(state.z());
            for (const auto& landmark : landmarks) {
                // Check if the landmark is of the same type as the observed intersection
                if (landmark.type == observed_intersection.type) {
                    // Calculate Euclidean distance between the observed intersection and the landmark
                    double distance = (landmark.position - Hfw * observed_intersection.rIWw.head<2>()).norm();

                    // If this landmark is closer update
                    if (distance < min_distance) {
                        min_distance              = distance;
                        closest_landmark_position = landmark.position;
                    }
                }
            }

            // If the closest landmark is too far away, return a landmark far away, which should have little effect
            if (min_distance > min_association_distance) {
                return Eigen::Vector2d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
            }

            // Transform the landmark position from the field to the world
            Eigen::Matrix<Scalar, 2, 1> rLWw = Hfw.inverse() * closest_landmark_position;
            return rLWw;
        }

        template <typename T, typename U>
        static auto difference(const T& a, const U& b) {
            return a - b;
        }

        // Getters
        [[nodiscard]] int get_rogue_count() const {
            return n_rogues;
        }
        [[nodiscard]] StateVec get_rogue() const {
            return reset_range;
        }

        [[nodiscard]] int get_particle_count() const {
            return n_particles;
        }
    };
}  // namespace module::localisation
#endif
