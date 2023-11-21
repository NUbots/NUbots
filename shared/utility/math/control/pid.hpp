#ifndef UTILITY_MATH_CONTROL_PID_HPP
#define UTILITY_MATH_CONTROL_PID_HPP

#include <Eigen/Dense>

namespace utility::math::control {

    template <typename Scalar, int N>
    class PID {
    private:
        Scalar Kp, Ki, Kd;
        Eigen::Matrix<Scalar, N, 1> integral_error;
        Eigen::Matrix<Scalar, N, 1> prev_error;
        Scalar max_integral = std::numeric_limits<Scalar>::max();
        Scalar min_integral = std::numeric_limits<Scalar>::lowest();

    public:
        PID() : Kp(0), Ki(0), Kd(0) {
            integral_error.setZero();
            prev_error.setZero();
        }

        PID(Scalar Kp, Scalar Ki, Scalar Kd, Scalar max_i_error, Scalar min_i_error)
            : Kp(Kp), Ki(Ki), Kd(Kd), max_integral(max_i_error), min_integral(min_i_error) {
            integral_error.setZero();
            prev_error.setZero();
        }

        Eigen::Matrix<Scalar, N, 1> update(const Eigen::Matrix<Scalar, N, 1>& setpoint,
                                           const Eigen::Matrix<Scalar, N, 1>& measured_value,
                                           Scalar dt) {
            Eigen::Matrix<Scalar, N, 1> error = setpoint - measured_value;

            // P term
            Eigen::Matrix<Scalar, N, 1> P = Kp * error;

            // I term
            integral_error += error * dt;
            for (int i = 0; i < N; ++i) {
                integral_error(i) = std::clamp(integral_error(i), min_integral, max_integral);
            }
            Eigen::Matrix<Scalar, N, 1> I = Ki * integral_error;

            // D term
            Eigen::Matrix<Scalar, N, 1> derivative = (error - prev_error) / dt;
            Eigen::Matrix<Scalar, N, 1> D          = Kd * derivative;

            prev_error = error;

            return P + I + D;
        }
    };

}  // namespace utility::math::control
#endif  // UTILITY_MATH_CONTROL_PID_HPP
