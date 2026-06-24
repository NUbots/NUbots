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

#ifndef MODULE_LOCALISATION_FIELDLOCALISATIONINTERSECTIONS_HPP
#define MODULE_LOCALISATION_FIELDLOCALISATIONINTERSECTIONS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include <random>
#include <vector>

#include "message/behaviour/state/Stability.hpp"
#include "message/input/Sensors.hpp"
#include "message/localisation/Field.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/vision/FieldIntersections.hpp"
#include "message/vision/FieldLines.hpp"
#include "message/vision/Goal.hpp"

#include "utility/localisation/OccupancyMap.hpp"
#include "utility/localisation/FieldLineOccupanyMap.hpp"

namespace module::localisation {

    class FieldLocalisationIntersections : public NUClear::Reactor {
    public:
        explicit FieldLocalisationIntersections(std::unique_ptr<NUClear::Environment> environment);

    private:
        struct Particle {
            Eigen::Vector3d state;  // [x, y, theta] representing Hfw
            double weight;
        };

        struct Config {
            int num_particles = 1000;
            int min_particles = 200;
            double grid_size = 0.05;
            bool save_map = false;
            std::string starting_side = "EITHER";
            std::chrono::seconds start_time_delay = std::chrono::seconds(5);
            Eigen::Vector3d initial_state = Eigen::Vector3d::Zero();
            int min_field_line_points = 30;
            double random_particle_injection_rate = 0.05;

            // Noise
            std::vector<double> process_noise{0.05, 0.05, 0.02, 0.1};
            Eigen::Vector3d base_noise = Eigen::Vector3d(0.005, 0.005, 0.001);

            // Likelihood
            double field_line_sigma = 0.2;
            double field_line_rand_prob = 0.05;
            double intersection_sigma = 0.3;
            double goal_post_sigma = 0.3;
        } cfg;

        // Particle filter state
        std::vector<Particle> particles;
        bool startup = true;
        Eigen::Isometry3d last_Hrw = Eigen::Isometry3d::Identity();
        bool last_Hrw_valid = false;

        // Random number generation
        std::mt19937 rng;

        // Map and landmarks
        utility::localisation::OccupancyMap<double> fieldline_distance_map;
        std::vector<utility::localisation::Landmark> landmarks;

        struct GoalPosts {
            Eigen::Vector3d left;
            Eigen::Vector3d right;
        } own_goal_posts, opp_goal_posts;

        // Methods
        void initialize_particles(const message::support::FieldDescription& fd);
        void time_update(const Eigen::Isometry3d& Hrw);
        void measurement_update(const std::shared_ptr<const message::vision::FieldIntersections>& intersections);
        void resample(const message::support::FieldDescription& fd);
        Eigen::Isometry3d compute_Hfw(const Eigen::Vector3d& state) const;
        double compute_particle_weight(const Eigen::Vector3d& state,
                                       const std::shared_ptr<const message::vision::FieldIntersections>& intersections);
    };

}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_FIELDLOCALISATIONINTERSECTIONS_HPP
