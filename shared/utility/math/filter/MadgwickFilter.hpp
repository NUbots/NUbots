#ifndef UTILITY_MATH_FILTER_MADGWICKFILTER_HPP
#define UTILITY_MATH_FILTER_MADGWICKFILTER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utility {
    namespace math {
        namespace filter {

            template <typename Scalar>
            class MadgwickFilter {
            private:
                Scalar beta;                            // Correction gain
                Scalar zeta;                            // Bias drift gain
                Eigen::Quaternion<Scalar> q;            // Orientation estimate
                Eigen::Matrix<Scalar, 3, 1> gyro_bias;  // Gyro bias estimate

            public:
                MadgwickFilter(Scalar beta = 0.05, Scalar zeta = 0.01)
                    : beta(beta), zeta(zeta), q(1, 0, 0, 0), gyro_bias(Eigen::Matrix<Scalar, 3, 1>::Zero()) {}

                void reset(const Eigen::Quaternion<Scalar>& q0 = Eigen::Quaternion<Scalar>::Identity()) {
                    q = q0;
                    gyro_bias.setZero();
                }

                void update(const Eigen::Matrix<Scalar, 3, 1>& raw_gyro,
                            const Eigen::Matrix<Scalar, 3, 1>& acc,
                            Scalar dt) {
                    // if (raw_acc.norm() < Scalar(1e-6))
                    //     return;

                    // Normalize accelerometer and flip Z so gravity points downward
                    // Eigen::Matrix<Scalar, 3, 1> acc = -raw_acc.normalized();

                    // Remove estimated gyro bias
                    Eigen::Matrix<Scalar, 3, 1> gyro = raw_gyro - gyro_bias;

                    // Current quaternion components
                    Scalar q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();

                    // Estimated gravity direction in body frame
                    Eigen::Matrix<Scalar, 3, 1> g;
                    g << 2 * (q1 * q3 - q0 * q2), 2 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

                    // Error between measured and estimated gravity
                    Eigen::Matrix<Scalar, 3, 1> error = g.cross(acc);

                    // Gyro bias correction
                    gyro_bias += zeta * error * dt;

                    // Quaternion derivative from gyro
                    Eigen::Quaternion<Scalar> q_dot;
                    q_dot.w() = 0.5 * (-q1 * gyro[0] - q2 * gyro[1] - q3 * gyro[2]);
                    q_dot.x() = 0.5 * (q0 * gyro[0] + q2 * gyro[2] - q3 * gyro[1]);
                    q_dot.y() = 0.5 * (q0 * gyro[1] - q1 * gyro[2] + q3 * gyro[0]);
                    q_dot.z() = 0.5 * (q0 * gyro[2] + q1 * gyro[1] - q2 * gyro[0]);

                    // Correction from accelerometer
                    Eigen::Quaternion<Scalar> correction(0, beta * error[0], beta * error[1], beta * error[2]);
                    q_dot.coeffs() -= correction.coeffs();

                    // Integrate
                    q.coeffs() += q_dot.coeffs() * dt;
                    q.normalize();
                }

                Eigen::Matrix<Scalar, 3, 3> get_rotation_matrix() const {
                    return q.toRotationMatrix();
                }

                Eigen::Quaternion<Scalar> get_quaternion() const {
                    return q;
                }

                Eigen::Matrix<Scalar, 3, 1> get_gyro_bias() const {
                    return gyro_bias;
                }

                void set_quaternion(const Eigen::Quaternion<Scalar>& new_q) {
                    q = new_q.normalized();
                }

                void set_gyro_bias(const Eigen::Matrix<Scalar, 3, 1>& new_bias) {
                    gyro_bias = new_bias;
                }
            };

        }  // namespace filter
    }      // namespace math
}  // namespace utility

#endif
