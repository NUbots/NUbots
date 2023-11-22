/*
 * Copyright (c) Hamburg Bit-Bots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
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
 *
 * This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
 * The original files can be found at:
 * https://github.com/Rhoban/model/
 */

#ifndef UTILITY_MOTION_SPLINES_TRAJECTORYUTILS_H
#define UTILITY_MOTION_SPLINES_TRAJECTORYUTILS_H

#include <Eigen/Core>
#include <nuclear>

#include "SmoothSpline.hpp"
#include "SplineContainer.hpp"

namespace utility::motion::splines {

    struct TrajectoryTypes {
        enum Value {
            UNKNOWN,
            IS_DOUBLE_SUPPORT,
            IS_LEFT_SUPPORT_FOOT,
            TRUNK_POS_X,
            TRUNK_POS_Y,
            TRUNK_POS_Z,
            TRUNK_AXIS_X,
            TRUNK_AXIS_Y,
            TRUNK_AXIS_Z,
            FOOT_POS_X,
            FOOT_POS_Y,
            FOOT_POS_Z,
            FOOT_AXIS_X,
            FOOT_AXIS_Y,
            FOOT_AXIS_Z
        };
        Value value = Value::UNKNOWN;


        // Constructors
        constexpr TrajectoryTypes() = default;
        constexpr TrajectoryTypes(uint8_t const& value_) : value(static_cast<Value>(value_)) {}
        constexpr TrajectoryTypes(Value const& value_) : value(value_) {}
        constexpr TrajectoryTypes(std::string const& str) {
            if (str == "IS_DOUBLE_SUPPORT") {
                value = Value::IS_DOUBLE_SUPPORT;
            }
            else if (str == "IS_LEFT_SUPPORT_FOOT") {
                value = Value::IS_LEFT_SUPPORT_FOOT;
            }
            else if (str == "TRUNK_POS_X") {
                value = Value::TRUNK_POS_X;
            }
            else if (str == "TRUNK_POS_Y") {
                value = Value::TRUNK_POS_Y;
            }
            else if (str == "TRUNK_POS_Z") {
                value = Value::TRUNK_POS_Z;
            }
            else if (str == "TRUNK_AXIS_X") {
                value = Value::TRUNK_AXIS_X;
            }
            else if (str == "TRUNK_AXIS_Y") {
                value = Value::TRUNK_AXIS_Y;
            }
            else if (str == "TRUNK_AXIS_Z") {
                value = Value::TRUNK_AXIS_Z;
            }
            else if (str == "FOOT_POS_X") {
                value = Value::FOOT_POS_X;
            }
            else if (str == "FOOT_POS_Y") {
                value = Value::FOOT_POS_Y;
            }
            else if (str == "FOOT_POS_Z") {
                value = Value::FOOT_POS_Z;
            }
            else if (str == "FOOT_AXIS_X") {
                value = Value::FOOT_AXIS_X;
            }
            else if (str == "FOOT_AXIS_Y") {
                value = Value::FOOT_AXIS_Y;
            }
            else if (str == "FOOT_AXIS_Z") {
                value = Value::FOOT_AXIS_Z;
            }
            else {
                throw std::runtime_error(fmt::format("String '{}' did not match any enum for TrajectoryTypes", str));
            }
        }

        // Operators
        [[nodiscard]] constexpr bool operator<(TrajectoryTypes const& other) const {
            return value < other.value;
        }
        [[nodiscard]] constexpr bool operator>(TrajectoryTypes const& other) const {
            return value > other.value;
        }
        [[nodiscard]] constexpr bool operator<=(TrajectoryTypes const& other) const {
            return value <= other.value;
        }
        [[nodiscard]] constexpr bool operator>=(TrajectoryTypes const& other) const {
            return value >= other.value;
        }
        [[nodiscard]] constexpr bool operator==(TrajectoryTypes const& other) const {
            return value == other.value;
        }
        [[nodiscard]] constexpr bool operator!=(TrajectoryTypes const& other) const {
            return value != other.value;
        }
        [[nodiscard]] constexpr bool operator<(TrajectoryTypes::Value const& other) const {
            return value < other;
        }
        [[nodiscard]] constexpr bool operator>(TrajectoryTypes::Value const& other) const {
            return value > other;
        }
        [[nodiscard]] constexpr bool operator<=(TrajectoryTypes::Value const& other) const {
            return value <= other;
        }
        [[nodiscard]] constexpr bool operator>=(TrajectoryTypes::Value const& other) const {
            return value >= other;
        }
        [[nodiscard]] constexpr bool operator==(TrajectoryTypes::Value const& other) const {
            return value == other;
        }
        [[nodiscard]] constexpr bool operator!=(TrajectoryTypes::Value const& other) const {
            return value != other;
        }

        // Conversions
        [[nodiscard]] constexpr operator Value() const {
            return value;
        }
        [[nodiscard]] constexpr operator uint8_t() const {
            return value;
        }

        [[nodiscard]] inline operator std::string() const {
            switch (value) {
                case Value::IS_DOUBLE_SUPPORT: return "IS_DOUBLE_SUPPORT";
                case Value::IS_LEFT_SUPPORT_FOOT: return "IS_LEFT_SUPPORT_FOOT";
                case Value::TRUNK_POS_X: return "TRUNK_POS_X";
                case Value::TRUNK_POS_Y: return "TRUNK_POS_Y";
                case Value::TRUNK_POS_Z: return "TRUNK_POS_Z";
                case Value::TRUNK_AXIS_X: return "TRUNK_AXIS_X";
                case Value::TRUNK_AXIS_Y: return "TRUNK_AXIS_Y";
                case Value::TRUNK_AXIS_Z: return "TRUNK_AXIS_Z";
                case Value::FOOT_POS_X: return "FOOT_POS_X";
                case Value::FOOT_POS_Y: return "FOOT_POS_Y";
                case Value::FOOT_POS_Z: return "FOOT_POS_Z";
                case Value::FOOT_AXIS_X: return "FOOT_AXIS_X";
                case Value::FOOT_AXIS_Y: return "FOOT_AXIS_Y";
                case Value::FOOT_AXIS_Z: return "FOOT_AXIS_Z";
                default:
                    throw std::runtime_error("enum TrajectoryTypes's value is corrupt, unknown value stored"
                                             + std::to_string(static_cast<uint8_t>(value)));
            }
        }
    };

    /**
     * @brief Tuple of 4 vectors, useful for the TrajectoriesTrunkFoot* functions
     */
    using Vector3fQuadruple = std::tuple<Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f>;

    /**
     * Simple typedef for trajectories container
     */
    using Trajectories = SplineContainer<SmoothSpline<float>, TrajectoryTypes, float>;

    /**
     * Return initialized trajectories for trunk/foot ik cartesian with empty splines
     */
    inline void trajectories_init(Trajectories& traj) {
        if (traj.size() != 0) {
            traj.reset();
        }
        else {
            traj.add(TrajectoryTypes::IS_DOUBLE_SUPPORT);
            traj.add(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT);
            traj.add(TrajectoryTypes::TRUNK_POS_X);
            traj.add(TrajectoryTypes::TRUNK_POS_Y);
            traj.add(TrajectoryTypes::TRUNK_POS_Z);
            traj.add(TrajectoryTypes::TRUNK_AXIS_X);
            traj.add(TrajectoryTypes::TRUNK_AXIS_Y);
            traj.add(TrajectoryTypes::TRUNK_AXIS_Z);
            traj.add(TrajectoryTypes::FOOT_POS_X);
            traj.add(TrajectoryTypes::FOOT_POS_Y);
            traj.add(TrajectoryTypes::FOOT_POS_Z);
            traj.add(TrajectoryTypes::FOOT_AXIS_X);
            traj.add(TrajectoryTypes::FOOT_AXIS_Y);
            traj.add(TrajectoryTypes::FOOT_AXIS_Z);
        }
    }

    /**
     * Compute from given spline container trajectory Cartesian trunk and foot position/velocity/acceleration
     * and assign it to given vector
     */
    [[nodiscard]] inline Vector3fQuadruple trajectories_trunk_foot_pos(const float& t, const Trajectories& traj) {
        // Compute Cartesian positions
        const auto trunk_pos = Eigen::Vector3f(traj.get(TrajectoryTypes::TRUNK_POS_X).pos(t),
                                               traj.get(TrajectoryTypes::TRUNK_POS_Y).pos(t),
                                               traj.get(TrajectoryTypes::TRUNK_POS_Z).pos(t));
        const auto trunkAxis = Eigen::Vector3f(traj.get(TrajectoryTypes::TRUNK_AXIS_X).pos(t),
                                               traj.get(TrajectoryTypes::TRUNK_AXIS_Y).pos(t),
                                               traj.get(TrajectoryTypes::TRUNK_AXIS_Z).pos(t));
        const auto footPos   = Eigen::Vector3f(traj.get(TrajectoryTypes::FOOT_POS_X).pos(t),
                                             traj.get(TrajectoryTypes::FOOT_POS_Y).pos(t),
                                             traj.get(TrajectoryTypes::FOOT_POS_Z).pos(t));
        const auto footAxis  = Eigen::Vector3f(traj.get(TrajectoryTypes::FOOT_AXIS_X).pos(t),
                                              traj.get(TrajectoryTypes::FOOT_AXIS_Y).pos(t),
                                              traj.get(TrajectoryTypes::FOOT_AXIS_Z).pos(t));
        return {trunk_pos, trunkAxis, footPos, footAxis};
    }

    [[nodiscard]] inline Vector3fQuadruple trajectoriesTrunkFootVel(const float& t, const Trajectories& traj) {
        // Compute Cartesian velocities
        const auto trunk_posVel = Eigen::Vector3f(traj.get(TrajectoryTypes::TRUNK_POS_X).vel(t),
                                                  traj.get(TrajectoryTypes::TRUNK_POS_Y).vel(t),
                                                  traj.get(TrajectoryTypes::TRUNK_POS_Z).vel(t));
        const auto trunkAxisVel = Eigen::Vector3f(traj.get(TrajectoryTypes::TRUNK_AXIS_X).vel(t),
                                                  traj.get(TrajectoryTypes::TRUNK_AXIS_Y).vel(t),
                                                  traj.get(TrajectoryTypes::TRUNK_AXIS_Z).vel(t));
        const auto footPosVel   = Eigen::Vector3f(traj.get(TrajectoryTypes::FOOT_POS_X).vel(t),
                                                traj.get(TrajectoryTypes::FOOT_POS_Y).vel(t),
                                                traj.get(TrajectoryTypes::FOOT_POS_Z).vel(t));
        const auto footAxisVel  = Eigen::Vector3f(traj.get(TrajectoryTypes::FOOT_AXIS_X).vel(t),
                                                 traj.get(TrajectoryTypes::FOOT_AXIS_Y).vel(t),
                                                 traj.get(TrajectoryTypes::FOOT_AXIS_Z).vel(t));
        return {trunk_posVel, trunkAxisVel, footPosVel, footAxisVel};
    }

    [[nodiscard]] inline Vector3fQuadruple trajectoriesTrunkFootAcc(const float& t, const Trajectories& traj) {
        // Compute Cartesian accelerations
        const auto trunk_posAcc = Eigen::Vector3f(traj.get(TrajectoryTypes::TRUNK_POS_X).acc(t),
                                                  traj.get(TrajectoryTypes::TRUNK_POS_Y).acc(t),
                                                  traj.get(TrajectoryTypes::TRUNK_POS_Z).acc(t));
        const auto trunkAxisAcc = Eigen::Vector3f(traj.get(TrajectoryTypes::TRUNK_AXIS_X).acc(t),
                                                  traj.get(TrajectoryTypes::TRUNK_AXIS_Y).acc(t),
                                                  traj.get(TrajectoryTypes::TRUNK_AXIS_Z).acc(t));
        const auto footPosAcc   = Eigen::Vector3f(traj.get(TrajectoryTypes::FOOT_POS_X).acc(t),
                                                traj.get(TrajectoryTypes::FOOT_POS_Y).acc(t),
                                                traj.get(TrajectoryTypes::FOOT_POS_Z).acc(t));
        const auto footAxisAcc  = Eigen::Vector3f(traj.get(TrajectoryTypes::FOOT_AXIS_X).acc(t),
                                                 traj.get(TrajectoryTypes::FOOT_AXIS_Y).acc(t),
                                                 traj.get(TrajectoryTypes::FOOT_AXIS_Z).acc(t));
        return {trunk_posAcc, trunkAxisAcc, footPosAcc, footAxisAcc};
    }

    /**
     * @brief Computes the support foot state, evaluating double support foot and whether is_left_supportFoot is true
     * at time t
     * @param t Time at which the state should be evaluated
     * @param traj The set of trajectories which are being evaluated
     * @return std::pair<bool, bool> {is_double_supportFoot, is_left_supportFoot}, as evaluated at time t
     */
    [[nodiscard]] constexpr inline std::pair<bool, bool> trajectories_support_foot_state(const float& t,
                                                                                         const Trajectories& traj) {
        // Compute support foot state
        return {traj.get(TrajectoryTypes::IS_DOUBLE_SUPPORT).pos(t) >= 0.5f,
                traj.get(TrajectoryTypes::IS_LEFT_SUPPORT_FOOT).pos(t) >= 0.5f};
    }

}  // namespace utility::motion::splines

#endif
