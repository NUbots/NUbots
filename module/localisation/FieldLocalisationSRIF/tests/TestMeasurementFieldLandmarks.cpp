/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
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

#include <Eigen/Core>
#include <catch2/catch_test_macros.hpp>
#include <string>
#include <utility>
#include <vector>

#include "measurement/MeasurementFieldLandmarks.hpp"
#include "srif/FieldMap.hpp"
#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/gaussian/GaussianInfo.hpp"
#include "utility/gaussian_filtering/rotation.hpp"

using module::localisation::measurement::MeasurementFieldLandmarks;
using module::localisation::srif::Detection;
using module::localisation::srif::FieldDimensions;
using module::localisation::srif::FieldMap;
using module::localisation::srif::LandmarkType;
using module::localisation::srif::SystemLocalisation;
using module::localisation::srif::VisionSample;
using utility::gaussian_filtering::Pose;
using utility::gaussian_filtering::rpy2quat;
using utility::gaussian_filtering::gaussian::GaussianInfo;

namespace {

    /// The default (5v5) FieldDescription.yaml dimensions
    FieldDimensions field_dimensions() {
        FieldDimensions dims;
        dims.line_width             = 0.06;
        dims.field_length           = 22.0;
        dims.field_width            = 14.0;
        dims.goal_depth             = 0.6;
        dims.goal_width             = 2.6;
        dims.goal_area_length       = 2.0;
        dims.goal_area_width        = 5.0;
        dims.penalty_mark_distance  = 3.5;
        dims.centre_circle_diameter = 4.0;
        dims.penalty_area_length    = 5.0;
        dims.penalty_area_width     = 8.0;
        dims.goalpost_width         = 0.10;
        dims.border_strip_min_width = 1.0;
        return dims;
    }

    /// A detection whose four corner rays all point at the landmark rLFf, as seen by a camera
    /// at Tfc
    Detection detection_of(const std::string& name, const Eigen::Vector3d& rLFf, const Pose<double>& Tfc) {
        const Eigen::Vector3d ray = (Tfc.rotation_matrix.transpose() * (rLFf - Tfc.translation_vector)).normalized();
        Detection det;
        det.name       = name;
        det.confidence = 0.9;
        det.corners    = ray.replicate<1, 4>();
        return det;
    }

}  // namespace

SCENARIO("Landmark detections associate under the class names the K1 YOLO model emits", "[slam][localisation]") {
    const FieldMap map(field_dimensions());

    // Torso on the field facing roughly down the field, with the camera above and ahead of it
    Eigen::VectorXd x = Eigen::VectorXd::Zero(SystemLocalisation::nx);
    x.head<3>() << -2.0, 1.0, 0.5;
    x.segment<4>(SystemLocalisation::i_quat) = rpy2quat(Eigen::Vector3d(0.0, 0.0, 0.3));
    const SystemLocalisation system(GaussianInfo<double>::from_sqrt_moment(
        x,
        Eigen::MatrixXd::Identity(SystemLocalisation::nx, SystemLocalisation::nx) * 0.01));

    Pose<double> Tbc;
    Tbc.rotation_matrix    = Eigen::Matrix3d::Identity();
    Tbc.translation_vector = Eigen::Vector3d(0.05, 0.0, 0.3);
    const Pose<double> Tfc = SystemLocalisation::field_pose<double>(x) * Tbc;

    // One detection per landmark type, each labelled with the given class names
    auto sample_labelled = [&](const std::vector<std::pair<LandmarkType, std::string>>& labels) {
        VisionSample sample;
        sample.t           = 0.0;
        sample.video_frame = -1;
        for (const auto& [type, name] : labels) {
            sample.detections.push_back(detection_of(name, map.landmarks(type).front(), Tfc));
        }
        return sample;
    };

    GIVEN("Detections labelled LCross, TCross, XCross and Goalpost") {
        const VisionSample sample = sample_labelled({{LandmarkType::L_INTERSECTION, "LCross"},
                                                     {LandmarkType::T_INTERSECTION, "TCross"},
                                                     {LandmarkType::X_INTERSECTION, "XCross"},
                                                     {LandmarkType::GOAL_POST, "Goalpost"}});

        THEN("Every one associates to a landmark") {
            const MeasurementFieldLandmarks measurement(0.0, sample, Tbc, map, system);
            CHECK(measurement.num_associated() == 4);
        }
    }

    GIVEN("The same detections labelled with classes that are not mapped landmarks") {
        const VisionSample sample = sample_labelled({{LandmarkType::L_INTERSECTION, "Ball"},
                                                     {LandmarkType::T_INTERSECTION, "Robot"},
                                                     {LandmarkType::X_INTERSECTION, "PenaltyPoint"},
                                                     {LandmarkType::GOAL_POST, "goal post"}});

        THEN("None associates") {
            const MeasurementFieldLandmarks measurement(0.0, sample, Tbc, map, system);
            CHECK(measurement.num_associated() == 0);
        }
    }
}
