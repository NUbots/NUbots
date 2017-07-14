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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_VISION_BALLDETECTOR_H
#define MODULES_VISION_BALLDETECTOR_H

#include <armadillo>
#include <nuclear>

#include "message/input/Image.h"
#include "message/vision/LookUpTable.h"

#include "utility/learning/KMeans.h"
#include "utility/math/geometry/Circle.h"
#include "utility/vision/LookUpTable.h"

namespace module {
namespace vision {

    class BallDetector : public NUClear::Reactor {
    private:
        uint MINIMUM_POINTS_FOR_CONSENSUS;
        uint MAXIMUM_ITERATIONS_PER_FITTING;
        uint MAXIMUM_FITTED_MODELS;

        double CONSENSUS_ERROR_THRESHOLD;
        double MAXIMUM_DISAGREEMENT_RATIO;

        double maximum_relative_seed_point_distance;

        double measurement_distance_variance_factor;
        double measurement_bearing_variance;
        double measurement_elevation_variance;

        double green_ratio_threshold;
        double green_radial_samples;
        double green_angular_samples;

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

        float approximateCircleGreenRatio(const utility::math::geometry::Circle& circle,
                                          const message::input::Image& image,
                                          const message::vision::LookUpTable& lut);

    public:
        /// @brief Called by the powerplant to build and setup the BallDetector reactor.
        explicit BallDetector(std::unique_ptr<NUClear::Environment> environment);
    };
}
}


#endif
