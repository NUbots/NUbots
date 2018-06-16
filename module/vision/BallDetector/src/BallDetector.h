/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_BALLDETECTOR_H
#define MODULES_VISION_BALLDETECTOR_H

#include <armadillo>
#include <nuclear>

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"
#include "message/vision/LookUpTable.h"
#include "message/vision/VisualMesh.h"

#include "utility/learning/KMeans.h"
#include "utility/math/geometry/Circle.h"
#include "utility/math/ransac/RansacConeModel.h"
#include "utility/math/ransac/RansacVisualMeshModel.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/vision/LookUpTable.h"

namespace module {
namespace vision {

    class BallDetector : public NUClear::Reactor {
    private:
        uint MINIMUM_POINTS_FOR_CONSENSUS;
        uint MAXIMUM_ITERATIONS_PER_FITTING;
        uint MAXIMUM_FITTED_MODELS;

        const double LAMBDA = 8.3116883;

        double CONSENSUS_ERROR_THRESHOLD;
        double MAXIMUM_DISAGREEMENT_RATIO;

        double mesh_seed_confidence_threshold;
        double mesh_branch_confidence_threshold;

        double maximum_relative_seed_point_distance;

        double measurement_distance_variance_factor;
        double measurement_bearing_variance;
        double measurement_elevation_variance;

        double green_ratio_threshold;
        double green_radial_samples;
        double green_angular_samples;

        arma::vec3 ball_angular_cov;

        utility::learning::KMeans kmeansClusterer;

        struct Frame {
            Frame() : time(), widthBall(arma::fill::zeros), projBall(arma::fill::zeros) {}
            Frame(const NUClear::clock::time_point& time, const arma::vec3& width, const arma::vec3& proj)
                : time(time), widthBall(width), projBall(proj) {}

            NUClear::clock::time_point time;
            arma::vec3 widthBall;
            arma::vec3 projBall;
        };
        Frame lastFrame;

        bool print_throwout_logs;
        bool print_mesh_debug;
        bool draw_cluster;

        float approximateCircleGreenRatio(const utility::math::ransac::RansacConeModel& cone,
                                          const message::input::Image& image,
                                          const message::vision::LookUpTable& lut,
                                          const message::input::CameraParameters& params);

        std::vector<std::vector<arma::vec4>> findClusters(const message::vision::VisualMesh& mesh,
                                                          const message::input::CameraParameters& cam);

    public:
        /// @brief Called by the powerplant to build and setup the BallDetector reactor.
        explicit BallDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace vision
}  // namespace module


#endif
