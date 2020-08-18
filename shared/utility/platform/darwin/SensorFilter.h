/*
 * This file is part of the NUbots Codebase.
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
#ifndef UTILITY_PLATFORM_DARWIN_SENSORFILTER_H
#define UTILITY_PLATFORM_DARWIN_SENSORFILTER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <memory>

#include "module/platform/darwin/SensorFilter/src/MotionModel.h"
#include "module/platform/darwin/SensorFilter/src/VirtualLoadSensor.h"

#include "message/input/Sensors.h"
#include "message/motion/BodySide.h"

#include "utility/input/ServoID.h"
#include "utility/math/filter/eigen/UKF.h"
#include "utility/motion/ForwardKinematics.h"

namespace utility {
namespace platform {
    namespace darwin {

        using message::input::Sensors;
        using message::motion::BodySide;
        using module::platform::darwin::MotionModel;
        using module::platform::darwin::MeasurementType::FLAT_FOOT_ODOMETRY;
        using module::platform::darwin::MeasurementType::FLAT_FOOT_ORIENTATION;

        using utility::input::ServoID;
        using utility::math::filter::UKF;

        template <typename Scalar>
        inline std::array<bool, 2> calculate_foot_down(const Sensors& sensors, const Scalar& certainty_threshold) {
            std::array<bool, 2> feet_down = {true};
            Eigen::Affine3d Htr(sensors.Htx[ServoID::R_ANKLE_ROLL]);
            Eigen::Affine3d Htl(sensors.Htx[ServoID::L_ANKLE_ROLL]);
            Eigen::Affine3d Hlr  = Htl.inverse() * Htr;
            Eigen::Vector3d rRLl = Hlr.translation();

            // Right foot is below left foot in left foot space
            if (rRLl.z() < -certainty_threshold) {
                feet_down[BodySide::RIGHT] = true;
                feet_down[BodySide::LEFT]  = false;
            }
            // Right foot is above left foot in left foot space
            else if (rRLl.z() > certainty_threshold) {
                feet_down[BodySide::RIGHT] = false;
                feet_down[BodySide::LEFT]  = true;
            }
            // Right foot and left foot are roughly the same height in left foot space
            else {
                feet_down[BodySide::RIGHT] = true;
                feet_down[BodySide::LEFT]  = true;
            }

            return feet_down;
        }

        template <typename Scalar>
        inline void update_flat_foot(UKF<Scalar, MotionModel>& filter,
                                     const bool& foot_down,
                                     bool& previous_foot_down,
                                     const Eigen::Matrix<Scalar, 3, 3>& odometry_noise,
                                     const Eigen::Matrix<Scalar, 4, 4>& orientation_noise,
                                     const Eigen::Transform<Scalar, 3, Eigen::Affine>& Htf,
                                     Eigen::Transform<Scalar, 3, Eigen::Affine>& Hwf) {
            if (foot_down && !previous_foot_down) {
                const auto& state           = filter.get();
                Eigen::Vector4d orientation = state.template segment<4>(MotionModel<Scalar>::QX);
                Eigen::Vector3d position    = state.template segment<3>(MotionModel<Scalar>::PX);

                Eigen::Affine3d Hwt;
                Hwt.linear()      = Eigen::Quaterniond(orientation).toRotationMatrix();
                Hwt.translation() = Eigen::Vector3d(position);

                Eigen::Affine3d Htg(utility::motion::kinematics::calculateGroundSpace(Htf, Hwt));

                Hwf                   = Hwt * Htg;
                Hwf.translation().z() = 0.0;

                previous_foot_down = true;
            }
            else if (foot_down && previous_foot_down) {
                // Use stored Hwf and Htf to calculate Hwt
                Eigen::Affine3d Hwt = Hwf * Htf.inverse();

                // do a foot based position update
                filter.measure(Eigen::Vector3d(Hwt.translation()), odometry_noise, FLAT_FOOT_ODOMETRY());

                // do a foot based orientation update
                Eigen::Quaterniond Rwt(Hwt.linear());
                filter.measure(Rwt.coeffs(), orientation_noise, FLAT_FOOT_ORIENTATION());
            }
            else if (!foot_down) {
                previous_foot_down = false;
            }
        }
    }  // namespace darwin
}  // namespace platform
}  // namespace utility

#endif  // UTILITY_PLATFORM_DARWIN_SENSORFILTER_H
