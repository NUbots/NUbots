#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "build.hpp"
#include "tasks.hpp"

namespace filter::kalman {
    void fInit_6DOF_GY_KALMAN(struct ::filter::tasks::SV_6DOF_GY_KALMAN& pthisSV);
    [[nodiscard]] Eigen::Quaternion<double> fRun_6DOF_GY_KALMAN(struct ::filter::tasks::SV_6DOF_GY_KALMAN& pthisSV,
                                                                const Eigen::Matrix<double, 3, 1>& accel_reading,
                                                                const Eigen::Matrix<double, 3, 1>& gyro_reading,
                                                                const int& ithisCoordSystem,
                                                                const bool& resetflag = false);
}  // namespace filter::kalman
#endif  // KALMAN_HPP