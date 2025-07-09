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

#include <nuclear>

#include "FieldLocalisationNLopt.hpp"

#include "utility/algorithm/assignment.hpp"

namespace module::localisation {

    using message::vision::FieldIntersections;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> FieldLocalisationNLopt::hungarian_association(
        const std::shared_ptr<const FieldIntersections>& field_intersections,
        const Eigen::Isometry3d& Hfw) {

        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> associations;

        // Create cost matrix for the Hungarian algorithm
        // Note that the assignment is intersection index to landmark index
        Eigen::MatrixXd cost_matrix(field_intersections->intersections.size(), landmarks.size());

        int intersection_idx = 0;
        for (const auto& intersection : field_intersections->intersections) {
            // Transform the detected intersection from world to field coordinates
            Eigen::Vector3d rIFf = Hfw * intersection.rIWw;

            for (size_t i = 0; i < landmarks.size(); ++i) {
                const auto& landmark = landmarks[i];

                // Default cost is infinite (invalid)
                double cost = std::numeric_limits<double>::max();

                // If the types match, check the distance
                if (landmark.type == intersection.type) {
                    double distance = (landmark.rLFf - rIFf).norm();

                    if (distance <= cfg.max_association_distance) {
                        cost = distance;
                    }
                }

                cost_matrix(intersection_idx, i) = cost;
            }

            intersection_idx++;
        }

        // Hungarian assignment
        auto assignment = utility::algorithm::determine_assignment(cost_matrix);

        // Build valid associations only
        for (const auto& [intersection_index, landmark_index] : assignment) {
            double cost = cost_matrix(intersection_index, landmark_index);

            if (cost < std::numeric_limits<double>::max()) {
                const auto& intersection = field_intersections->intersections.at(intersection_index);
                const auto& landmark     = landmarks.at(landmark_index);

                Eigen::Vector3d rIFf = Hfw * intersection.rIWw;
                associations.emplace_back(landmark.rLFf, rIFf);
            }
            else {
                log<INFO>("Rejected assignment due to excessive cost: intersection ",
                          intersection_index,
                          " ↔ landmark ",
                          landmark_index,
                          " (cost = ∞)");
            }
        }

        return associations;
    }


    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> FieldLocalisationNLopt::greedy_association(
        const std::shared_ptr<const FieldIntersections>& field_intersections,
        const Eigen::Isometry3d& Hfw) {

        std::vector<Eigen::Vector3d> occupied_landmarks{};
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> associations;

        // Greedily associate each intersection with the closest landmark
        for (const auto& intersection : field_intersections->intersections) {
            double min_distance = std::numeric_limits<double>::max();
            Eigen::Vector3d closest_landmark;
            bool found_association = false;

            // Transform the detected intersection from world to field coordinates
            Eigen::Vector3d rIFf = Hfw * intersection.rIWw;

            for (const auto& landmark : landmarks) {
                // If the landmark is the same type as our measurement and we haven't already assigned it
                if (landmark.type == intersection.type
                    && std::find(occupied_landmarks.begin(), occupied_landmarks.end(), landmark.rLFf)
                           == occupied_landmarks.end()) {
                    // Calculate Euclidean distance between the detected intersection and the landmark
                    double distance = (landmark.rLFf - rIFf).norm();

                    // If this landmark is closer and within the maximum association distance, update the
                    // association
                    if (distance < min_distance && distance <= cfg.max_association_distance) {
                        min_distance      = distance;
                        closest_landmark  = landmark.rLFf;
                        found_association = true;
                    }
                }
            }

            // Mark the closest landmark as occupied if within the distance threshold
            if (found_association) {
                occupied_landmarks.push_back(closest_landmark);
                associations.emplace_back(closest_landmark, rIFf);
            }
        }

        return associations;
    }

}  // namespace module::localisation
