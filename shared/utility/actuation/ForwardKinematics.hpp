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

#ifndef UTILITY_ACTUATION_FORWARDKINEMATICS_HPP
#define UTILITY_ACTUATION_FORWARDKINEMATICS_HPP

#include <Eigen/Geometry>
#include <cmath>
#include <nuclear>
#include <vector>

#include "message/actuation/BodySide.hpp"
#include "message/actuation/KinematicsModel.hpp"
#include "message/input/Sensors.hpp"

#include "utility/input/LimbID.hpp"
#include "utility/input/ServoID.hpp"
#include "utility/math/angle.hpp"

namespace utility::actuation::kinematics {

    using message::actuation::BodySide;
    using message::actuation::KinematicsModel;
    using message::input::Sensors;
    using utility::input::LimbID;
    using utility::input::ServoID;

    template <typename Scalar>
    [[nodiscard]] inline std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> calculateHeadJointPosition(
        const KinematicsModel& model,
        const Scalar& HEAD_PITCH,
        const Scalar& HEAD_YAW,
        const ServoID& servoID) {
        std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> positions{};

        Eigen::Transform<Scalar, 3, Eigen::Isometry> runningTransform =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        const Eigen::Matrix<Scalar, 3, 1> NECK_POS(model.head.NECK_BASE_POS_FROM_ORIGIN_X,
                                                   model.head.NECK_BASE_POS_FROM_ORIGIN_Y,
                                                   model.head.NECK_BASE_POS_FROM_ORIGIN_Z);
        const Scalar NECK_LENGTH = model.head.NECK_LENGTH;

        // Translate to base of neck from origin
        runningTransform = runningTransform.translate(NECK_POS);
        // Rotate to face out of base of neck
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(Scalar(-M_PI_2), Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Rotate head in yaw axis
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(HEAD_YAW, Eigen::Matrix<Scalar, 3, 1>::UnitX()));
        // Translate to top of neck (i.e. next motor axle)
        runningTransform = runningTransform.translate(Eigen::Matrix<Scalar, 3, 1>(NECK_LENGTH, Scalar(0), Scalar(0)));
        // YAW
        // Return the basis pointing out of the top of the torso with z pointing out the back of the neck. Pos
        // is top of neck (at hip pitch motor)
        positions[ServoID::HEAD_YAW] = runningTransform;
        if (servoID == ServoID::HEAD_YAW) {
            return positions;
        }

        // Rotate to face forward direction of neck
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(Scalar(M_PI_2), Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Rotate pitch
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(HEAD_PITCH, Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // PITCH
        // Return basis pointing along camera vector (ie x is camera vector, z out of top of head). Pos at
        // camera position
        positions[ServoID::HEAD_PITCH] = runningTransform;
        return positions;
    }

    template <typename Scalar>
    [[nodiscard]] inline std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>>
        calculateHeadJointPosition(const KinematicsModel& model, const Sensors& sensors, const ServoID& servoID) {
        return calculateHeadJointPosition<Scalar>(model,
                                                  sensors.servo[static_cast<int>(ServoID::HEAD_PITCH)].present_position,
                                                  sensors.servo[static_cast<int>(ServoID::HEAD_YAW)].present_position,
                                                  servoID);
    }
    /*! @brief
        @NOTE read " runningTransform *= utility::math::matrix::_RotationMatrix(angle, 4); " as "Rotate the
       running transform about its local _ coordinate by angle."
        @return Returns basis matrix for position of end of limb controlled by the specified motor.

        The basis 'faces' down its x axis.
    */
    template <typename Scalar>
    [[nodiscard]] inline std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> calculateLegJointPosition(
        const KinematicsModel& model,
        const Sensors& sensors,
        const ServoID& servoID,
        const BodySide& isLeft) {
        std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> positions{};
        Eigen::Transform<Scalar, 3, Eigen::Isometry> runningTransform =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        // Variables to mask left and right leg differences:
        ServoID HIP_YAW;
        ServoID HIP_ROLL;
        ServoID HIP_PITCH;
        ServoID KNEE;
        ServoID ANKLE_PITCH;
        ServoID ANKLE_ROLL;
        int negativeIfRight = 1;

        if (isLeft == BodySide::LEFT) {
            HIP_YAW     = ServoID::L_HIP_YAW;
            HIP_ROLL    = ServoID::L_HIP_ROLL;
            HIP_PITCH   = ServoID::L_HIP_PITCH;
            KNEE        = ServoID::L_KNEE;
            ANKLE_PITCH = ServoID::L_ANKLE_PITCH;
            ANKLE_ROLL  = ServoID::L_ANKLE_ROLL;
        }
        else {
            HIP_YAW         = ServoID::R_HIP_YAW;
            HIP_ROLL        = ServoID::R_HIP_ROLL;
            HIP_PITCH       = ServoID::R_HIP_PITCH;
            KNEE            = ServoID::R_KNEE;
            ANKLE_PITCH     = ServoID::R_ANKLE_PITCH;
            ANKLE_ROLL      = ServoID::R_ANKLE_ROLL;
            negativeIfRight = -1;
        }

        // Hip pitch
        runningTransform =
            runningTransform.translate(Eigen::Matrix<Scalar, 3, 1>(model.leg.HIP_OFFSET_X,
                                                                   negativeIfRight * model.leg.HIP_OFFSET_Y,
                                                                   -model.leg.HIP_OFFSET_Z));
        // Rotate to face down the leg (see above for definitions of terms, including 'facing')
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(M_PI_2, Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Using right hand rule along global z gives positive direction of yaw:
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxis<Scalar>(-sensors.servo[HIP_YAW].present_position, Eigen::Matrix<Scalar, 3, 1>::UnitX()));
        // Return basis facing from body to hip centre (down) with z aligned with the axis of the hip roll motor
        // axis. Position at hip joint
        positions[HIP_YAW] = runningTransform;
        if (servoID == HIP_YAW) {
            return positions;
        }

        runningTransform = runningTransform.rotate(
            Eigen::AngleAxis<Scalar>(sensors.servo[HIP_ROLL].present_position, Eigen::Matrix<Scalar, 3, 1>::UnitZ()));
        // Return basis facing down leg plane, with z oriented through axis of roll motor. Position still hip
        // joint
        positions[HIP_ROLL] = runningTransform;
        if (servoID == HIP_ROLL) {
            return positions;
        }

        // Rotate to face down upper leg
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxis<Scalar>(sensors.servo[HIP_PITCH].present_position, Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Translate down upper leg
        runningTransform =
            runningTransform.translate(Eigen::Matrix<Scalar, 3, 1>(model.leg.UPPER_LEG_LENGTH, 0.0, 0.0));
        // Return basis faces down upper leg, with z out of front of thigh. Pos = knee axis centre
        positions[HIP_PITCH] = runningTransform;
        if (servoID == HIP_PITCH) {
            return positions;
        }


        // Rotate to face down lower leg
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxis<Scalar>(sensors.servo[KNEE].present_position, Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Translate down lower leg
        runningTransform =
            runningTransform.translate(Eigen::Matrix<Scalar, 3, 1>(model.leg.LOWER_LEG_LENGTH, 0.0, 0.0));
        // Return basis facing down lower leg, with z out of front of shin. Pos = ankle axis centre
        positions[KNEE] = runningTransform;
        if (servoID == KNEE) {
            return positions;
        }


        // Rotate to face down foot (pitch)
        runningTransform = runningTransform.rotate(Eigen::AngleAxis<Scalar>(sensors.servo[ANKLE_PITCH].present_position,
                                                                            Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Return basis facing pitch down to foot with z out the front of the foot. Pos = ankle axis centre
        positions[ANKLE_PITCH] = runningTransform;
        if (servoID == ANKLE_PITCH) {
            return positions;
        }

        // Rotate to face down foot (roll)
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxis<Scalar>(sensors.servo[ANKLE_ROLL].present_position, Eigen::Matrix<Scalar, 3, 1>::UnitZ()));
        // Rotate so x faces toward toes
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(Scalar(-M_PI_2), Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Translate to ground
        runningTransform =
            runningTransform.translate(Eigen::Matrix<Scalar, 3, 1>(Scalar(0), Scalar(0), -model.leg.FOOT_HEIGHT));
        // Return basis with x out of the front of the toe and z out the top of foot. Pos = ankle axis centre
        // projected to ground
        positions[ANKLE_ROLL] = runningTransform;
        return positions;
    }

    /*! @brief
        @NOTE read " runningTransform *= utility::math::matrix::_RotationMatrix(angle, 4); " as "Rotate the
       running transform about its local _ coordinate by angle."
        @return Returns basis matrix for position of end of limb controlled by the specified motor.

        The basis 'faces' down its x axis.
    */
    template <typename Scalar>
    [[nodiscard]] inline std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> calculateArmJointPosition(
        const KinematicsModel& model,
        const Sensors& sensors,
        const ServoID& servoID,
        const BodySide& isLeft) {
        std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> positions{};
        Eigen::Transform<Scalar, 3, Eigen::Isometry> runningTransform =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();
        // Variables to mask left and right differences:
        ServoID SHOULDER_PITCH;
        ServoID SHOULDER_ROLL;
        ServoID ELBOW;
        int negativeIfRight = 1;

        if (isLeft == BodySide::LEFT) {
            SHOULDER_PITCH = ServoID::L_SHOULDER_PITCH;
            SHOULDER_ROLL  = ServoID::L_SHOULDER_ROLL;
            ELBOW          = ServoID::L_ELBOW;
        }
        else {
            SHOULDER_PITCH  = ServoID::R_SHOULDER_PITCH;
            SHOULDER_ROLL   = ServoID::R_SHOULDER_ROLL;
            ELBOW           = ServoID::R_ELBOW;
            negativeIfRight = -1;
        }

        const Scalar& shoulder_pitch = sensors.servo[SHOULDER_PITCH].present_position;
        const Scalar& shoulder_roll  = sensors.servo[SHOULDER_ROLL].present_position;
        const Scalar& elbow          = sensors.servo[ELBOW].present_position;

        // Translate to shoulder
        runningTransform = runningTransform.translate(
            Eigen::Matrix<Scalar, 3, 1>(model.arm.SHOULDER_X_OFFSET,
                                        negativeIfRight * model.arm.DISTANCE_BETWEEN_SHOULDERS * 0.5,
                                        model.arm.SHOULDER_Z_OFFSET));
        // Rotate about shoulder pitch with zero position Zombie arms
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxis<Scalar>(shoulder_pitch - Scalar(M_PI_2), Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Translate to end of shoulder part
        runningTransform =
            runningTransform.translate(Eigen::Matrix<Scalar, 3, 1>(model.arm.SHOULDER_LENGTH,
                                                                   negativeIfRight * model.arm.SHOULDER_WIDTH,
                                                                   -model.arm.SHOULDER_HEIGHT));
        // Return matrix pointing forward out of shoulder, y same as global y. Pos = at centre of shoulder roll
        // joint
        positions[SHOULDER_PITCH] = runningTransform;
        if (servoID == SHOULDER_PITCH) {
            return positions;
        }

        // Rotate by the shoulder roll
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(shoulder_roll, Eigen::Matrix<Scalar, 3, 1>::UnitX()));
        // Translate to centre of next joint
        runningTransform =
            runningTransform.translate(Eigen::Matrix<Scalar, 3, 1>(model.arm.UPPER_ARM_X_OFFSET,
                                                                   negativeIfRight * model.arm.UPPER_ARM_Y_OFFSET,
                                                                   -model.arm.UPPER_ARM_LENGTH));
        // Rotate to face down arm
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(Scalar(M_PI_2), Eigen::Matrix<Scalar, 3, 1>::UnitY()));

        positions[SHOULDER_ROLL] = runningTransform;
        if (servoID == SHOULDER_ROLL) {
            return positions;
        }

        // Rotate by the elbow angle
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxis<Scalar>(elbow, Eigen::Matrix<Scalar, 3, 1>::UnitY()));
        // Translate to centre of end of arm, in line with joint
        runningTransform =
            runningTransform.translate(Eigen::Matrix<Scalar, 3, 1>(model.arm.LOWER_ARM_LENGTH,
                                                                   negativeIfRight * model.arm.LOWER_ARM_Y_OFFSET,
                                                                   -model.arm.LOWER_ARM_Z_OFFSET));
        positions[ELBOW] = runningTransform;
        return positions;
    }

    /*! @brief
     */
    template <typename Scalar>
    [[nodiscard]] inline std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>>
        calculatePosition(const KinematicsModel& model, const Sensors& sensors, const ServoID& servoID) {
        switch (servoID.value) {
            case ServoID::HEAD_YAW:
            case ServoID::HEAD_PITCH: return calculateHeadJointPosition<Scalar>(model, sensors, servoID);
            case ServoID::R_SHOULDER_PITCH:
            case ServoID::R_SHOULDER_ROLL:
            case ServoID::R_ELBOW: return calculateArmJointPosition<Scalar>(model, sensors, servoID, BodySide::RIGHT);
            case ServoID::L_SHOULDER_PITCH:
            case ServoID::L_SHOULDER_ROLL:
            case ServoID::L_ELBOW: return calculateArmJointPosition<Scalar>(model, sensors, servoID, BodySide::LEFT);
            case ServoID::R_HIP_YAW:
            case ServoID::R_HIP_ROLL:
            case ServoID::R_HIP_PITCH:
            case ServoID::R_KNEE:
            case ServoID::R_ANKLE_PITCH:
            case ServoID::R_ANKLE_ROLL:
                return calculateLegJointPosition<Scalar>(model, sensors, servoID, BodySide::RIGHT);
            case ServoID::L_HIP_YAW:
            case ServoID::L_HIP_ROLL:
            case ServoID::L_HIP_PITCH:
            case ServoID::L_KNEE:
            case ServoID::L_ANKLE_PITCH:
            case ServoID::L_ANKLE_ROLL:
                return calculateLegJointPosition<Scalar>(model, sensors, servoID, BodySide::LEFT);
            default: return std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>>();
        }
    }

    template <typename Scalar>
    [[nodiscard]] inline std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> calculateAllPositions(
        const KinematicsModel& model,
        const Sensors& sensors) {
        std::map<ServoID, Eigen::Transform<Scalar, 3, Eigen::Isometry>> result{};
        for (const auto& r : calculatePosition<Scalar>(model, sensors, ServoID::L_ANKLE_ROLL)) {
            result[r.first] = r.second;
        }
        for (const auto& r : calculatePosition<Scalar>(model, sensors, ServoID::R_ANKLE_ROLL)) {
            result[r.first] = r.second;
        }
        for (const auto& r : calculatePosition<Scalar>(model, sensors, ServoID::HEAD_PITCH)) {
            result[r.first] = r.second;
        }
        for (const auto& r : calculatePosition<Scalar>(model, sensors, ServoID::L_ELBOW)) {
            result[r.first] = r.second;
        }
        for (const auto& r : calculatePosition<Scalar>(model, sensors, ServoID::R_ELBOW)) {
            result[r.first] = r.second;
        }

        return result;
    }
    /*! @brief Adds up the mass vectors stored in the robot model and normalises the resulting position
        @return [x_com, y_com, z_com, total_mass] relative to the torso basis
    */
    template <typename Scalar>
    [[nodiscard]] inline Eigen::Matrix<Scalar, 4, 1> calculateCentreOfMass(
        const message::actuation::KinematicsModel& model,
        const std::array<Eigen::Matrix<Scalar, 4, 4>, 20>& Htx) {

        // Convenience function to transform particle-space CoM to torso-space CoM
        // Htx - transform from particle space to torso space
        // particle - CoM coordinates in particle space
        auto com = [](const Eigen::Matrix<Scalar, 4, 4>& Htx, const Eigen::Matrix<Scalar, 4, 1>& particle) {
            // Split out CoM and mass
            Eigen::Matrix<Scalar, 4, 1> com(particle.x(), particle.y(), particle.z(), Scalar(1));
            const Scalar mass = particle.w();

            // Calculate CoM in torso space
            com = Htx * com;

            return std::pair<Eigen::Matrix<Scalar, 3, 1>, Scalar>{Eigen::Matrix<Scalar, 3, 1>(com(1), com(2), com(3)),
                                                                  mass};
        };

        // Get the centre of mass for each particle in torso space
        // There are 16 particles in total
        std::array<std::pair<Eigen::Matrix<Scalar, 3, 1>, Scalar>, 16> particles = {
            com(Htx[utility::input::ServoID::HEAD_PITCH], model.mass_model.head),
            com(Htx[utility::input::ServoID::L_SHOULDER_PITCH], model.mass_model.arm_upper),
            com(Htx[utility::input::ServoID::R_SHOULDER_PITCH], model.mass_model.arm_upper),
            com(Htx[utility::input::ServoID::L_SHOULDER_ROLL], model.mass_model.arm_lower),
            com(Htx[utility::input::ServoID::R_SHOULDER_ROLL], model.mass_model.arm_lower),
            com(Htx[utility::input::ServoID::L_HIP_ROLL], model.mass_model.hip_block),
            com(Htx[utility::input::ServoID::R_HIP_ROLL], model.mass_model.hip_block),
            com(Htx[utility::input::ServoID::L_HIP_PITCH], model.mass_model.leg_upper),
            com(Htx[utility::input::ServoID::R_HIP_PITCH], model.mass_model.leg_upper),
            com(Htx[utility::input::ServoID::L_KNEE], model.mass_model.leg_lower),
            com(Htx[utility::input::ServoID::R_KNEE], model.mass_model.leg_lower),
            com(Htx[utility::input::ServoID::L_ANKLE_PITCH], model.mass_model.ankle_block),
            com(Htx[utility::input::ServoID::R_ANKLE_PITCH], model.mass_model.ankle_block),
            com(Htx[utility::input::ServoID::L_ANKLE_ROLL], model.mass_model.foot),
            com(Htx[utility::input::ServoID::R_ANKLE_ROLL], model.mass_model.foot),
            std::pair<Eigen::Matrix<Scalar, 3, 1>, Scalar>{Eigen::Matrix<Scalar, 3, 1>{model.mass_model.torso.x(),
                                                                                       model.mass_model.torso.y(),
                                                                                       model.mass_model.torso.z()},
                                                           model.mass_model.torso.w()},
        };

        // Calculate the CoM for the entire robot
        std::pair<Eigen::Matrix<Scalar, 3, 1>, Scalar> robot_com = {Eigen::Matrix<Scalar, 3, 1>::Zero(), Scalar(0)};
        for (const auto& particle : particles) {
            robot_com.first = (robot_com.first * robot_com.second + particle.first * particle.second)
                              / (robot_com.second + particle.second);
            robot_com.second = robot_com.second + particle.second;
        }

        return Eigen::Matrix<Scalar, 4, 1>(robot_com.first(1),
                                           robot_com.first(2),
                                           robot_com.first(3),
                                           robot_com.second);
    }

    /*! @brief Transforms inertial tensors for each robot particle into torso space and sums to find the total
       inertial tensor
        @return [[xx, xy, xz], relative to the torso basis
                 [xy, yy, yz],
                 [xz, yz, zz]]
    */
    template <typename Scalar>
    [[nodiscard]] inline Eigen::Matrix<Scalar, 3, 3> calculateInertialTensor(
        const message::actuation::KinematicsModel& model,
        const std::array<Eigen::Matrix<Scalar, 4, 4>, 20>& Htx) {

        // Convenience function to transform particle-space inertial tensors to torso-space inertial tensor
        // Htx - transform from particle space to torso space
        // particle - CoM coordinates in particle space
        auto translateTensor = [](const Eigen::Matrix<Scalar, 4, 4>& Htx,
                                  const Eigen::Matrix<Scalar, 3, 3>& tensor,
                                  const Eigen::Matrix<Scalar, 4, 1>& com_mass) {
            Eigen::Matrix<Scalar, 4, 1> com(com_mass.x(), com_mass.y(), com_mass.z(), 1.0);
            com = Htx * com;

            // Calculate distance to particle CoM from particle origin, using skew-symmetric matrix
            const Scalar x = com.x();
            const Scalar y = com.y();
            const Scalar z = com.z();
            Eigen::Matrix<Scalar, 3, 3> d;
            // clang-format off
                    d <<  y * y + z * z, -x * y,         -x * z,
                         -x * y,          x * x + z * z, -y * z,
                         -x * z,         -y * z,          x * x + y * y;
            // clang-format on

            // We need to rotate the tensor into our torso reference frame
            // https://en.wikipedia.org/wiki/Moment_of_inertia#Body_frame
            Eigen::Matrix<Scalar, 3, 3> torso_tensor =
                Htx.block(0, 0, 3, 3) * tensor * Htx.block(0, 0, 3, 3).transpose();

            // Translate tensor using the parallel axis theorem
            Eigen::Matrix<Scalar, 3, 3> tensor_com = com_mass.w() * (torso_tensor - d);

            return tensor_com;
        };

        // Get the centre of mass for each particle in torso space
        // There are 16 particles in total
        std::array<Eigen::Matrix<Scalar, 3, 3>, 16> particles = {
            translateTensor(Htx[utility::input::ServoID::HEAD_PITCH], model.tensor_model.head, model.mass_model.head),
            translateTensor(Htx[utility::input::ServoID::L_SHOULDER_PITCH],
                            model.tensor_model.arm_upper,
                            model.mass_model.arm_upper),
            translateTensor(Htx[utility::input::ServoID::R_SHOULDER_PITCH],
                            model.tensor_model.arm_upper,
                            model.mass_model.arm_upper),
            translateTensor(Htx[utility::input::ServoID::L_SHOULDER_ROLL],
                            model.tensor_model.arm_lower,
                            model.mass_model.arm_lower),
            translateTensor(Htx[utility::input::ServoID::R_SHOULDER_ROLL],
                            model.tensor_model.arm_lower,
                            model.mass_model.arm_lower),
            translateTensor(Htx[utility::input::ServoID::L_HIP_ROLL],
                            model.tensor_model.hip_block,
                            model.mass_model.hip_block),
            translateTensor(Htx[utility::input::ServoID::R_HIP_ROLL],
                            model.tensor_model.hip_block,
                            model.mass_model.hip_block),
            translateTensor(Htx[utility::input::ServoID::L_HIP_PITCH],
                            model.tensor_model.leg_upper,
                            model.mass_model.leg_upper),
            translateTensor(Htx[utility::input::ServoID::R_HIP_PITCH],
                            model.tensor_model.leg_upper,
                            model.mass_model.leg_upper),
            translateTensor(Htx[utility::input::ServoID::L_ANKLE_PITCH],
                            model.tensor_model.leg_lower,
                            model.mass_model.leg_lower),
            translateTensor(Htx[utility::input::ServoID::R_ANKLE_PITCH],
                            model.tensor_model.leg_lower,
                            model.mass_model.leg_lower),
            translateTensor(Htx[utility::input::ServoID::L_ANKLE_PITCH],
                            model.tensor_model.ankle_block,
                            model.mass_model.ankle_block),
            translateTensor(Htx[utility::input::ServoID::R_ANKLE_PITCH],
                            model.tensor_model.ankle_block,
                            model.mass_model.ankle_block),
            translateTensor(Htx[utility::input::ServoID::L_ANKLE_ROLL], model.tensor_model.foot, model.mass_model.foot),
            translateTensor(Htx[utility::input::ServoID::R_ANKLE_ROLL], model.tensor_model.foot, model.mass_model.foot),
            model.tensor_model.torso};

        // Calculate the inertial tensor for the entire robot
        Eigen::Matrix<Scalar, 3, 3> inertia_tensor = Eigen::Matrix<Scalar, 3, 3>::Zero();
        for (const auto& particle : particles) {
            inertia_tensor += particle;
        }

        return inertia_tensor;
    }  // namespace kinematics

    template <typename T, typename Scalar = typename T::Scalar, typename MatrixType = typename T::LinearMatrixType>
    [[nodiscard]] inline T calculateGroundSpace(const T& Htf, const T& Hwt) {
        // Retrieve rotations needed for creating the space
        // support foot to torso rotation, and world to torso rotation
        const MatrixType Rtf(Htf.rotation());

        // Fix the foot in world space
        const MatrixType Rwf(Hwt.rotation() * Rtf);

        // Dot product of foot z (in world space) with world z
        const Scalar alpha = utility::math::angle::acos_clamped(Rwf(2, 2));

        Eigen::Matrix<Scalar, 3, 1> axis(Rwf.col(2).cross(Eigen::Matrix<Scalar, 3, 1>::UnitZ()).normalized());

        // Axis angle is foot to ground
        const MatrixType Rwg(Eigen::AngleAxis<Scalar>(alpha, axis).toRotationMatrix() * Rwf);
        const MatrixType Rtg(Hwt.rotation().transpose() * Rwg);

        // Ground space assemble!
        T Htg;
        Htg.linear()      = Rtg;
        Htg.translation() = Htf.translation();

        return Htg;
    }
}  // namespace utility::actuation::kinematics

#endif  // UTILITY_ACTUATION_FORWARDKINEMATICS_HPP
