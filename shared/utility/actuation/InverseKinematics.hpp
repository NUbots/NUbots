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

#ifndef UTILITY_ACTUATION_INVERSEKINEMATICS_HPP
#define UTILITY_ACTUATION_INVERSEKINEMATICS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <limits>
#include <nuclear>
#include <vector>

#include "message/actuation/KinematicsModel.hpp"
#include "message/input/Sensors.hpp"

#include "utility/actuation/ForwardKinematics.hpp"
#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"
#include "utility/math/coordinates.hpp"

namespace utility::actuation::kinematics {

    using utility::input::LimbID;
    using utility::input::ServoID;
    using utility::math::angle::acos_clamped;
    using utility::math::angle::angle_between;

    /** @brief Calculates the leg joints for a given input ankle position. The robot coordinate system has origin a
     * distance DISTANCE_FROM_BODY_TO_HIP_JOINT above the midpoint of the hips. Robot coordinate system: x is out of the
     * front of the robot y is left, from right shoulder to left z is upward, from feet to head Input ankle coordinate
     * system: x is forward, from heel to toe y is left, z is normal to the plane of the foot
     * @param model KinematicsModel containing dimensions of the robot
     * @param Htf_ Desired pose of foot {f} in torso {t} space
     * @param limb Which leg to solve IK for (left or right)
     * @return Vector of pairs of servo ID and joint angles
     */
    template <typename Scalar>
    [[nodiscard]] std::vector<std::pair<ServoID, Scalar>> calculate_leg_joints(
        const message::actuation::KinematicsModel& model,
        const Eigen::Transform<Scalar, 3, Eigen::Isometry>& Htf_,
        const LimbID& limb) {
        std::vector<std::pair<ServoID, Scalar>> joints;

        // Correct for input referencing the bottom of the foot and convention used in IK implementation
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Htf(Htf_);
        Htf = Htf.translate(Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, model.leg.FOOT_HEIGHT));
        Eigen::Matrix<Scalar, 3, 3> R;
        // clang-format off
        R << 0.0, 1.0,  0.0,
                1.0, 0.0,  0.0,
                0.0, 0.0, -1.0;
        // clang-format on
        Htf.translation() = R * Htf.translation();
        Htf.linear()      = R * Htf.linear() * R.transpose();

        // Swap legs if needed
        if (limb != LimbID::LEFT_LEG) {
            Htf(0, 1)             = -Htf(0, 1);
            Htf(0, 2)             = -Htf(0, 2);
            Htf(1, 0)             = -Htf(1, 0);
            Htf(2, 0)             = -Htf(2, 0);
            Htf.translation().x() = -Htf.translation().x();
        }

        // Apply offsets to Htf and clamp to limits
        const Eigen::Matrix<Scalar, 3, 1> ankle_x = Htf.matrix().template leftCols<1>().template head<3>();
        const Eigen::Matrix<Scalar, 3, 1> ankle_y = Htf.matrix().template middleCols<1>(1).template head<3>();
        Eigen::Matrix<Scalar, 3, 1> rFTt          = Htf.translation();

        const Eigen::Matrix<Scalar, 3, 1> hip_offset(model.leg.LENGTH_BETWEEN_LEGS * 0.5,
                                                     model.leg.HIP_OFFSET_X,
                                                     model.leg.HIP_OFFSET_Z);
        rFTt = rFTt - hip_offset;

        const Scalar max_leg_length = model.leg.UPPER_LEG_LENGTH + model.leg.LOWER_LEG_LENGTH;
        Scalar length               = rFTt.norm();
        if (length > max_leg_length) {
            rFTt = rFTt * max_leg_length / length;
        }

        // Knee pitch
        const Scalar sqr_length    = std::pow(length, 2);
        const Scalar sqr_upper_leg = std::pow(model.leg.UPPER_LEG_LENGTH, 2);
        const Scalar sqr_lower_leg = std::pow(model.leg.LOWER_LEG_LENGTH, 2);
        const Scalar cos_knee      = (sqr_upper_leg + sqr_lower_leg - sqr_length)
                                / (2.0 * model.leg.UPPER_LEG_LENGTH * model.leg.LOWER_LEG_LENGTH);
        const Scalar knee_pitch = M_PI - acos_clamped(cos_knee);

        // Ankle pitch
        const Scalar cos_lower_leg =
            (sqr_lower_leg + sqr_length - sqr_upper_leg) / (2.0 * model.leg.LOWER_LEG_LENGTH * length);
        const Scalar lower_leg   = acos_clamped(cos_lower_leg);
        const Scalar phi2        = acos_clamped(rFTt.dot(ankle_y) / length);
        const Scalar ankle_pitch = -(lower_leg + phi2 - M_PI_2);

        // Ankle roll
        const Eigen::Matrix<Scalar, 3, 1> uFTt  = rFTt.normalized();
        const Eigen::Matrix<Scalar, 3, 1> hip_x = Eigen::Matrix<Scalar, 3, 1>(ankle_y.cross(uFTt)).normalized();
        // Will be unit as ankle_y and hip_x are normal and unit
        const Eigen::Matrix<Scalar, 3, 1> leg_plane_tangent = ankle_y.cross(hip_x);
        const Scalar ankle_roll = std::atan2(ankle_x.dot(leg_plane_tangent), ankle_x.dot(hip_x));

        // Hip roll
        const bool is_ankle_above_waist = uFTt.z() < 0;
        const Scalar cos_z_and_hip_x    = Eigen::Matrix<Scalar, 3, 1>::UnitZ().dot(hip_x);
        const bool hip_roll_positive    = cos_z_and_hip_x <= 0;
        Eigen::Matrix<Scalar, 3, 1> leg_plane_global_z =
            (is_ankle_above_waist ? -1 : 1) * (Eigen::Matrix<Scalar, 3, 1>::UnitZ() - (cos_z_and_hip_x * hip_x));
        const Scalar leg_plane_global_z_length = leg_plane_global_z.norm();
        if (leg_plane_global_z_length > 0) {
            leg_plane_global_z /= leg_plane_global_z_length;
        }
        const Scalar hip_roll =
            (hip_roll_positive ? 1 : -1) * angle_between(leg_plane_global_z, Eigen::Matrix<Scalar, 3, 1>::UnitZ());

        // Hip pitch
        const Scalar phi4               = knee_pitch - lower_leg;
        const Scalar sin_pi_minus_phi_2 = std::sin(M_PI - phi2);
        const Eigen::Matrix<Scalar, 3, 1> unit_upper_leg =
            uFTt * (std::sin(phi2 - phi4) / sin_pi_minus_phi_2) + ankle_y * (std::sin(phi4) / sin_pi_minus_phi_2);
        const bool is_hip_pitch_positive = hip_x.dot(unit_upper_leg.cross(leg_plane_global_z)) >= 0;
        const Scalar hip_pitch = (is_hip_pitch_positive ? -1 : 1) * angle_between(unit_upper_leg, leg_plane_global_z);

        // Hip yaw
        // If leg is above waist then hip_x is pointing in the wrong direction in the xy plane
        Eigen::Matrix<Scalar, 3, 1> hip_x_projected = (is_ankle_above_waist ? -1 : 1) * hip_x;
        hip_x_projected.z()                         = 0;
        hip_x_projected.normalize();
        const bool is_hip_yaw_positive = hip_x_projected.y() >= 0;
        const Scalar hip_yaw =
            (is_hip_yaw_positive ? -1 : 1) * angle_between(hip_x_projected, Eigen::Matrix<Scalar, 3, 1>::UnitX());

        if (limb == LimbID::LEFT_LEG) {
            joints.emplace_back(ServoID::L_HIP_YAW, hip_yaw);
            joints.emplace_back(ServoID::L_HIP_ROLL, hip_roll);
            joints.emplace_back(ServoID::L_HIP_PITCH, hip_pitch);
            joints.emplace_back(ServoID::L_KNEE, knee_pitch);
            joints.emplace_back(ServoID::L_ANKLE_PITCH, ankle_pitch);
            joints.emplace_back(ServoID::L_ANKLE_ROLL, ankle_roll);
        }
        else {
            joints.emplace_back(ServoID::R_HIP_YAW, model.leg.LEFT_TO_RIGHT_HIP_YAW * hip_yaw);
            joints.emplace_back(ServoID::R_HIP_ROLL, model.leg.LEFT_TO_RIGHT_HIP_ROLL * hip_roll);
            joints.emplace_back(ServoID::R_HIP_PITCH, model.leg.LEFT_TO_RIGHT_HIP_PITCH * hip_pitch);
            joints.emplace_back(ServoID::R_KNEE, model.leg.LEFT_TO_RIGHT_KNEE * knee_pitch);
            joints.emplace_back(ServoID::R_ANKLE_PITCH, model.leg.LEFT_TO_RIGHT_ANKLE_PITCH * ankle_pitch);
            joints.emplace_back(ServoID::R_ANKLE_ROLL, model.leg.LEFT_TO_RIGHT_ANKLE_ROLL * ankle_roll);
        }

        return joints;
    }

    /**
     * @brief Calculates the head joint angles to "look" in a desired direction
     * @param uPCt Desired direction to look, unit vector from camera {C} to point {P} in torso {t} space
     * @return Vector of pairs of servo ID and joint angles
     */
    template <typename Scalar>
    [[nodiscard]] std::vector<std::pair<ServoID, Scalar>> calculate_head_joints(
        const Eigen::Matrix<Scalar, 3, 1>& uPCt) {
        return {std::make_pair(ServoID::HEAD_YAW, std::atan2(uPCt.y(), uPCt.x())),
                std::make_pair(ServoID::HEAD_PITCH,
                               std::atan2(-uPCt.z(), std::sqrt(uPCt.x() * uPCt.x() + uPCt.y() * uPCt.y())))};
    }
}  // namespace utility::actuation::kinematics

#endif  // UTILITY_ACTUATION_INVERSEKINEMATICS_HPP
