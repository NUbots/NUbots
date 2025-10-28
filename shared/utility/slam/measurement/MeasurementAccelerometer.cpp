#include "MeasurementAccelerometer.hpp"

#include <cmath>

#include "../rotation.hpp"  // For rpy2rot and rotation utilities

namespace utility::slam::measurement {

    MeasurementAccelerometer::MeasurementAccelerometer(double time, const Eigen::VectorXd& measurement)
        : MeasurementGaussianLikelihood(time, measurement) {
        updateMethod_ = UpdateMethod::AFFINE;
    }

    MeasurementAccelerometer::MeasurementAccelerometer(double time, const Eigen::VectorXd& measurement, int verbosity)
        : MeasurementGaussianLikelihood(time, measurement, verbosity) {
        updateMethod_ = UpdateMethod::AFFINE;
    }

    MeasurementAccelerometer::~MeasurementAccelerometer() = default;

    std::string MeasurementAccelerometer::getProcessString() const {
        return "Accelerometer measurement update:";
    }

    // Evaluate h(x) from the measurement model y = h(x) + v
    // For accelerometer: h(x) = R_nb * g_n (assuming quasi-static, a_n ≈ 0)
    // where g_n = [0, 0, g] in NED frame
    Eigen::VectorXd MeasurementAccelerometer::predict(const Eigen::VectorXd& state_vector,
                                                      const system::SystemEstimator& /*system*/) const {
        // Extract orientation (Euler angles) from state
        Eigen::Vector3d Thetanb = state_vector.segment<3>(9);  // [roll, pitch, yaw]

        // Convert Euler angles to rotation matrix R_nb (navigation to body)
        Eigen::Matrix3d R_nb = utility::slam::rpy2rot(Thetanb);

        // Gravity vector in navigation frame (NED): [0, 0, g]
        Eigen::Vector3d gravity_nav(0.0, 0.0, gravity_);

        // Predicted accelerometer measurement (assuming quasi-static conditions)
        // y = R_nb * g_n
        Eigen::Vector3d predicted_measurement = R_nb.transpose() * gravity_nav;

        return predicted_measurement;
    }

    // Evaluate h(x) and its Jacobian J = dh/dx
    Eigen::VectorXd MeasurementAccelerometer::predict(const Eigen::VectorXd& state_vector,
                                                      const system::SystemEstimator& system,
                                                      Eigen::MatrixXd& jacobian) const {
        // Get prediction at current state
        Eigen::VectorXd predicted_measurement = predict(state_vector, system);

        // Compute Jacobian using finite differences
        const double epsilon = 1e-7;

        // Initialize full Jacobian
        jacobian.resize(3, state_vector.size());
        jacobian.setZero();

        // Only orientation states (9-11) affect the accelerometer measurement
        for (int j = 9; j < 12; ++j) {
            // Create perturbed state
            Eigen::VectorXd state_perturbed = state_vector;
            state_perturbed(j) += epsilon;

            // Compute prediction at perturbed state
            Eigen::VectorXd h_perturbed = predict(state_perturbed, system);

            // Finite difference: dh/dx_j ≈ (h(x + ε*e_j) - h(x)) / ε
            jacobian.col(j) = (h_perturbed - predicted_measurement) / epsilon;
        }

        return predicted_measurement;
    }

    // Evaluate h(x), Jacobian, and Hessian
    Eigen::VectorXd MeasurementAccelerometer::predict(const Eigen::VectorXd& state_vector,
                                                      const system::SystemEstimator& system,
                                                      Eigen::MatrixXd& jacobian,
                                                      Eigen::Tensor<double, 3>& hessian) const {
        // Get prediction and Jacobian
        Eigen::VectorXd predicted_measurement = predict(state_vector, system, jacobian);

        // Compute Hessian using finite differences on the Jacobian
        // d²h_i/dx_j dx_k ≈ (dh_i/dx_k(x + ε*e_j) - dh_i/dx_k(x)) / ε
        const double epsilon = 1e-7;

        hessian.resize(3, state_vector.size(), state_vector.size());
        hessian.setZero();

        // Only orientation states (9-11) have non-zero second derivatives
        for (int j = 9; j < 12; ++j) {
            // Create perturbed state
            Eigen::VectorXd state_perturbed = state_vector;
            state_perturbed(j) += epsilon;

            // Compute Jacobian at perturbed state
            Eigen::MatrixXd jacobian_perturbed;
            predict(state_perturbed, system, jacobian_perturbed);

            // Compute second derivative: d²h/dx_j dx_k
            for (int k = 9; k < 12; ++k) {
                for (int i = 0; i < 3; ++i) {  // Loop over measurement dimensions
                    // Finite difference of Jacobian
                    hessian(i, j, k) = (jacobian_perturbed(i, k) - jacobian(i, k)) / epsilon;
                }
            }
        }

        return predicted_measurement;
    }

    // Define measurement noise covariance
    gaussian::GaussianInfo<double> MeasurementAccelerometer::noiseDensity(
        const system::SystemEstimator& /*system*/) const {

        // sqrt_cov is an upper triangular matrix such that sqrt_cov^T * sqrt_cov = R
        // where R is the measurement noise covariance
        Eigen::MatrixXd sqrt_cov = sigma_accel_ * Eigen::MatrixXd::Identity(3, 3);

        return gaussian::GaussianInfo<double>::fromSqrtMoment(sqrt_cov);
    }

}  // namespace utility::slam::measurement
