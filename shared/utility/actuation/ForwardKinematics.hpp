/*
 * MIT License
 *
 * Copyright (c) 2014 NUbots
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


    [[nodiscard]] inline std::map<ServoID, Eigen::Isometry3d> calculateHeadJointPosition(const KinematicsModel& model,
                                                                                         const float& HEAD_PITCH,
                                                                                         const float& HEAD_YAW,
                                                                                         const ServoID& servoID) {
        std::map<ServoID, Eigen::Isometry3d> positions{};

        Eigen::Isometry3d runningTransform = Eigen::Isometry3d::Identity();
        const Eigen::Vector3d NECK_POS(model.head.NECK_BASE_POS_FROM_ORIGIN_X,
                                       model.head.NECK_BASE_POS_FROM_ORIGIN_Y,
                                       model.head.NECK_BASE_POS_FROM_ORIGIN_Z);
        const float NECK_LENGTH = model.head.NECK_LENGTH;

        // Translate to base of neck from origin
        runningTransform = runningTransform.translate(NECK_POS);
        // Rotate to face out of base of neck
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
        // Rotate head in yaw axis
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(HEAD_YAW, Eigen::Vector3d::UnitX()));
        // Translate to top of neck (i.e. next motor axle)
        runningTransform = runningTransform.translate(Eigen::Vector3d(NECK_LENGTH, 0.0, 0.0));
        // YAW
        // Return the basis pointing out of the top of the torso with z pointing out the back of the neck. Pos
        // is top of neck (at hip pitch motor)
        positions[ServoID::HEAD_YAW] = runningTransform;
        if (servoID == ServoID::HEAD_YAW) {
            return positions;
        }

        // Rotate to face forward direction of neck
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
        // Rotate pitch
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(HEAD_PITCH, Eigen::Vector3d::UnitY()));
        // PITCH
        // Return basis pointing along camera vector (ie x is camera vector, z out of top of head). Pos at
        // camera position
        positions[ServoID::HEAD_PITCH] = runningTransform;
        return positions;
    }

    [[nodiscard]] inline std::map<ServoID, Eigen::Isometry3d> calculateHeadJointPosition(const KinematicsModel& model,
                                                                                         const Sensors& sensors,
                                                                                         const ServoID& servoID) {
        return calculateHeadJointPosition(model,
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
    [[nodiscard]] inline std::map<ServoID, Eigen::Isometry3d> calculateLegJointPosition(const KinematicsModel& model,
                                                                                        const Sensors& sensors,
                                                                                        const ServoID& servoID,
                                                                                        const BodySide& isLeft) {
        std::map<ServoID, Eigen::Isometry3d> positions{};
        Eigen::Isometry3d runningTransform = Eigen::Isometry3d::Identity();
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
        runningTransform = runningTransform.translate(
            Eigen::Vector3d(model.leg.HIP_OFFSET_X, negativeIfRight * model.leg.HIP_OFFSET_Y, -model.leg.HIP_OFFSET_Z));
        // Rotate to face down the leg (see above for definitions of terms, including 'facing')
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));
        // Using right hand rule along global z gives positive direction of yaw:
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxisd(-sensors.servo[HIP_YAW].present_position, Eigen::Vector3d::UnitX()));
        // Return basis facing from body to hip centre (down) with z aligned with the axis of the hip roll motor
        // axis. Position at hip joint
        positions[HIP_YAW] = runningTransform;
        if (servoID == HIP_YAW) {
            return positions;
        }

        runningTransform = runningTransform.rotate(
            Eigen::AngleAxisd(sensors.servo[HIP_ROLL].present_position, Eigen::Vector3d::UnitZ()));
        // Return basis facing down leg plane, with z oriented through axis of roll motor. Position still hip
        // joint
        positions[HIP_ROLL] = runningTransform;
        if (servoID == HIP_ROLL) {
            return positions;
        }

        // Rotate to face down upper leg
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxisd(sensors.servo[HIP_PITCH].present_position, Eigen::Vector3d::UnitY()));
        // Translate down upper leg
        runningTransform = runningTransform.translate(Eigen::Vector3d(model.leg.UPPER_LEG_LENGTH, 0.0, 0.0));
        // Return basis faces down upper leg, with z out of front of thigh. Pos = knee axis centre
        positions[HIP_PITCH] = runningTransform;
        if (servoID == HIP_PITCH) {
            return positions;
        }


        // Rotate to face down lower leg
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxisd(sensors.servo[KNEE].present_position, Eigen::Vector3d::UnitY()));
        // Translate down lower leg
        runningTransform = runningTransform.translate(Eigen::Vector3d(model.leg.LOWER_LEG_LENGTH, 0.0, 0.0));
        // Return basis facing down lower leg, with z out of front of shin. Pos = ankle axis centre
        positions[KNEE] = runningTransform;
        if (servoID == KNEE) {
            return positions;
        }


        // Rotate to face down foot (pitch)
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxisd(sensors.servo[ANKLE_PITCH].present_position, Eigen::Vector3d::UnitY()));
        // Return basis facing pitch down to foot with z out the front of the foot. Pos = ankle axis centre
        positions[ANKLE_PITCH] = runningTransform;
        if (servoID == ANKLE_PITCH) {
            return positions;
        }

        // Rotate to face down foot (roll)
        runningTransform = runningTransform.rotate(
            Eigen::AngleAxisd(sensors.servo[ANKLE_ROLL].present_position, Eigen::Vector3d::UnitZ()));
        // Rotate so x faces toward toes
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));
        // Translate to ground
        runningTransform = runningTransform.translate(Eigen::Vector3d(0.0, 0.0, -model.leg.FOOT_HEIGHT));
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
    [[nodiscard]] inline std::map<ServoID, Eigen::Isometry3d> calculateArmJointPosition(const KinematicsModel& model,
                                                                                        const Sensors& sensors,
                                                                                        const ServoID& servoID,
                                                                                        const BodySide& isLeft) {
        std::map<ServoID, Eigen::Isometry3d> positions{};
        Eigen::Isometry3d runningTransform = Eigen::Isometry3d::Identity();
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

        const float& shoulder_pitch = sensors.servo[SHOULDER_PITCH].present_position;
        const float& shoulder_roll  = sensors.servo[SHOULDER_ROLL].present_position;
        const float& elbow          = sensors.servo[ELBOW].present_position;

        // Translate to shoulder
        runningTransform =
            runningTransform.translate(Eigen::Vector3d(model.arm.SHOULDER_X_OFFSET,
                                                       negativeIfRight * model.arm.DISTANCE_BETWEEN_SHOULDERS * 0.5,
                                                       model.arm.SHOULDER_Z_OFFSET));
        // Rotate about shoulder pitch with zero position Zombie arms
        runningTransform =
            runningTransform.rotate(Eigen::AngleAxisd(shoulder_pitch - M_PI_2, Eigen::Vector3d::UnitY()));
        // Translate to end of shoulder part
        runningTransform = runningTransform.translate(Eigen::Vector3d(model.arm.SHOULDER_LENGTH,
                                                                      negativeIfRight * model.arm.SHOULDER_WIDTH,
                                                                      -model.arm.SHOULDER_HEIGHT));
        // Return matrix pointing forward out of shoulder, y same as global y. Pos = at centre of shoulder roll
        // joint
        positions[SHOULDER_PITCH] = runningTransform;
        if (servoID == SHOULDER_PITCH) {
            return positions;
        }

        // Rotate by the shoulder roll
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(shoulder_roll, Eigen::Vector3d::UnitX()));
        // Translate to centre of next joint
        runningTransform = runningTransform.translate(Eigen::Vector3d(model.arm.UPPER_ARM_X_OFFSET,
                                                                      negativeIfRight * model.arm.UPPER_ARM_Y_OFFSET,
                                                                      -model.arm.UPPER_ARM_LENGTH));
        // Rotate to face down arm
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()));

        positions[SHOULDER_ROLL] = runningTransform;
        if (servoID == SHOULDER_ROLL) {
            return positions;
        }

        // Rotate by the elbow angle
        runningTransform = runningTransform.rotate(Eigen::AngleAxisd(elbow, Eigen::Vector3d::UnitY()));
        // Translate to centre of end of arm, in line with joint
        runningTransform = runningTransform.translate(Eigen::Vector3d(model.arm.LOWER_ARM_LENGTH,
                                                                      negativeIfRight * model.arm.LOWER_ARM_Y_OFFSET,
                                                                      -model.arm.LOWER_ARM_Z_OFFSET));
        positions[ELBOW] = runningTransform;
        return positions;
    }

    /*! @brief
     */
    [[nodiscard]] inline std::map<ServoID, Eigen::Isometry3d> calculatePosition(const KinematicsModel& model,
                                                                                const Sensors& sensors,
                                                                                const ServoID& servoID) {
        switch (servoID.value) {
            case ServoID::HEAD_YAW:
            case ServoID::HEAD_PITCH: return calculateHeadJointPosition(model, sensors, servoID);
            case ServoID::R_SHOULDER_PITCH:
            case ServoID::R_SHOULDER_ROLL:
            case ServoID::R_ELBOW: return calculateArmJointPosition(model, sensors, servoID, BodySide::RIGHT);
            case ServoID::L_SHOULDER_PITCH:
            case ServoID::L_SHOULDER_ROLL:
            case ServoID::L_ELBOW: return calculateArmJointPosition(model, sensors, servoID, BodySide::LEFT);
            case ServoID::R_HIP_YAW:
            case ServoID::R_HIP_ROLL:
            case ServoID::R_HIP_PITCH:
            case ServoID::R_KNEE:
            case ServoID::R_ANKLE_PITCH:
            case ServoID::R_ANKLE_ROLL: return calculateLegJointPosition(model, sensors, servoID, BodySide::RIGHT);
            case ServoID::L_HIP_YAW:
            case ServoID::L_HIP_ROLL:
            case ServoID::L_HIP_PITCH:
            case ServoID::L_KNEE:
            case ServoID::L_ANKLE_PITCH:
            case ServoID::L_ANKLE_ROLL: return calculateLegJointPosition(model, sensors, servoID, BodySide::LEFT);
            default: return std::map<ServoID, Eigen::Isometry3d>();
        }
    }

    [[nodiscard]] inline std::map<ServoID, Eigen::Isometry3d> calculateAllPositions(const KinematicsModel& model,
                                                                                    const Sensors& sensors) {
        std::map<ServoID, Eigen::Isometry3d> result{};
        for (const auto& r : calculatePosition(model, sensors, ServoID::L_ANKLE_ROLL)) {
            result[r.first] = r.second;
        }
        for (const auto& r : calculatePosition(model, sensors, ServoID::R_ANKLE_ROLL)) {
            result[r.first] = r.second;
        }
        for (const auto& r : calculatePosition(model, sensors, ServoID::HEAD_PITCH)) {
            result[r.first] = r.second;
        }
        for (const auto& r : calculatePosition(model, sensors, ServoID::L_ELBOW)) {
            result[r.first] = r.second;
        }
        for (const auto& r : calculatePosition(model, sensors, ServoID::R_ELBOW)) {
            result[r.first] = r.second;
        }

        return result;
    }
    /*! @brief Adds up the mass vectors stored in the robot model and normalises the resulting position
        @return [x_com, y_com, z_com, total_mass] relative to the torso basis
    */
    [[nodiscard]] inline Eigen::Vector4d calculateCentreOfMass(const message::actuation::KinematicsModel& model,
                                                               const std::array<Eigen::Matrix4d, 20>& Htx) {

        // Convenience function to transform particle-space CoM to torso-space CoM
        // Htx - transform from particle space to torso space
        // particle - CoM coordinates in particle space
        auto com = [](const Eigen::Matrix4d& Htx, const Eigen::Vector4d& particle) {
            // Split out CoM and mass
            Eigen::Vector4d com(particle.x(), particle.y(), particle.z(), 1.0);
            const double mass = particle.w();

            // Calculate CoM in torso space
            com = Htx * com;

            return std::pair<Eigen::Vector3d, double>{Eigen::Vector3d(com.x(), com.y(), com.z()), mass};
        };

        // Get the centre of mass for each particle in torso space
        // There are 16 particles in total
        std::array<std::pair<Eigen::Vector3d, double>, 16> particles = {
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
            std::pair<Eigen::Vector3d, double>{
                Eigen::Vector3d{model.mass_model.torso.x(), model.mass_model.torso.y(), model.mass_model.torso.z()},
                model.mass_model.torso.w()},
        };

        // Calculate the CoM for the entire robot
        std::pair<Eigen::Vector3d, double> robot_com = {Eigen::Vector3d::Zero(), 0.0};
        for (const auto& particle : particles) {
            robot_com.first = (robot_com.first * robot_com.second + particle.first * particle.second)
                              / (robot_com.second + particle.second);
            robot_com.second = robot_com.second + particle.second;
        }

        return Eigen::Vector4d{robot_com.first.x(), robot_com.first.y(), robot_com.first.z(), robot_com.second};
    }

    /*! @brief Transforms inertial tensors for each robot particle into torso space and sums to find the total
       inertial tensor
        @return [[xx, xy, xz], relative to the torso basis
                 [xy, yy, yz],
                 [xz, yz, zz]]
    */
    [[nodiscard]] inline Eigen::Matrix3d calculateInertialTensor(const message::actuation::KinematicsModel& model,
                                                                 const std::array<Eigen::Matrix4d, 20>& Htx) {

        // Convenience function to transform particle-space inertial tensors to torso-space inertial tensor
        // Htx - transform from particle space to torso space
        // particle - CoM coordinates in particle space
        auto translateTensor = [](const Eigen::Matrix4d& Htx,
                                  const Eigen::Matrix3d& tensor,
                                  const Eigen::Vector4d& com_mass) {
            Eigen::Vector4d com(com_mass.x(), com_mass.y(), com_mass.z(), 1.0);
            com = Htx * com;

            // Calculate distance to particle CoM from particle origin, using skew-symmetric matrix
            const double x = com.x();
            const double y = com.y();
            const double z = com.z();
            Eigen::Matrix3d d;
            // clang-format off
                    d <<  y * y + z * z, -x * y,         -x * z,
                         -x * y,          x * x + z * z, -y * z,
                         -x * z,         -y * z,          x * x + y * y;
            // clang-format on

            // We need to rotate the tensor into our torso reference frame
            // https://en.wikipedia.org/wiki/Moment_of_inertia#Body_frame
            Eigen::Matrix3d torso_tensor = Htx.topLeftCorner<3, 3>() * tensor * Htx.topLeftCorner<3, 3>().transpose();

            // Translate tensor using the parallel axis theorem
            Eigen::Matrix3d tensor_com = com_mass.w() * (torso_tensor - d);

            return tensor_com;
        };

        // Get the centre of mass for each particle in torso space
        // There are 16 particles in total
        std::array<Eigen::Matrix3d, 16> particles = {
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
        Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Zero();
        for (const auto& particle : particles) {
            inertia_tensor += particle;
        }

        return inertia_tensor;
    }  // namespace kinematics

    [[nodiscard]] inline Eigen::Matrix2d calculateRobotToIMU(const Eigen::Isometry3d& orientation) {
        const Eigen::Vector3d xRobotImu  = orientation.rotation().topRows<1>();
        const Eigen::Vector2d projXRobot = xRobotImu.head<2>().normalized();
        const Eigen::Vector2d projYRobot = Eigen::Vector2d(-projXRobot.y(), projXRobot.x());

        Eigen::Matrix2d robotToImu;
        robotToImu << projXRobot, projYRobot;

        return robotToImu;
    }

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
