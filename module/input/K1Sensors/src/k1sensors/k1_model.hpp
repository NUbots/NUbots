#ifndef MODULE_INPUT_K1SENSORS_K1MODEL_HPP
#define MODULE_INPUT_K1SENSORS_K1MODEL_HPP

#include <Eigen/Geometry>
#include <cmath>
#include <memory>

#include "message/input/Sensors.hpp"

namespace module::input {

    /// @brief Computes forward kinematics from Trunk to Head_2 (pitch link).
    ///
    /// The K1 head kinematic chain (from URDF):
    ///   Trunk -> AAHead_yaw (revolute, axis z, origin [0.0056, 0, 0.2149])
    ///          -> Head_1
    ///          -> Head_pitch (revolute, axis y, origin [0, 0, 0.033])
    ///          -> Head_2
    ///
    /// @param sensors Sensors message containing current servo positions
    /// @return Htp — transform from the Head_2 pitch link to the Trunk (base)
    inline Eigen::Isometry3d compute_Htp(const std::unique_ptr<message::input::Sensors>& sensors) {
        // Head yaw and pitch joint angles from servo positions
        const double q_yaw   = sensors->servo.at(18).present_position;  // HEAD_YAW = 18
        const double q_pitch = sensors->servo.at(19).present_position;  // HEAD_PITCH = 19

        // Joint 1: AAHead_yaw — fixed translation then rotation about z
        const double cy = std::cos(q_yaw);
        const double sy = std::sin(q_yaw);

        // Joint 2: Head_pitch — fixed translation then rotation about y
        const double cp = std::cos(q_pitch);
        const double sp = std::sin(q_pitch);

        // Htp = T1(translate + Rz(yaw)) * T2(translate + Ry(pitch))
        // Combined rotation: Rz(yaw) * Ry(pitch)
        Eigen::Isometry3d Htp = Eigen::Isometry3d::Identity();
        Htp.linear() << cy * cp, -sy, cy * sp,  //
            sy * cp, cy, sy * sp,                //
            -sp, 0.0, cp;

        // Translation: t1 + Rz(yaw) * t2
        // t1 = [0.0056, 0, 0.2149], t2 = [0, 0, 0.033]
        // Rz(yaw) * [0, 0, 0.033] = [0, 0, 0.033] (z-axis rotation doesn't affect z-aligned vector)
        Htp.translation() << 0.0056, 0.0, 0.2149 + 0.033;

        return Htp;
    }

}  // namespace module::input

#endif  // MODULE_INPUT_K1SENSORS_K1MODEL_HPP
