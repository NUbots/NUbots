#include "MeasurementGyroscope.hpp"
#include <cmath>

namespace utility::slam::measurement {

    MeasurementGyroscope::MeasurementGyroscope(double time, const Eigen::VectorXd& y)
        : MeasurementGaussianLikelihood(time, y) {
        updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;
    }

    MeasurementGyroscope::MeasurementGyroscope(double time, const Eigen::VectorXd& y, int verbosity)
        : MeasurementGaussianLikelihood(time, y, verbosity) {
        updateMethod_ = UpdateMethod::BFGSTRUSTSQRT;
    }

    MeasurementGyroscope::~MeasurementGyroscope() = default;

    std::string MeasurementGyroscope::getProcessString() const {
        return "Gyroscope measurement update:";
    }

    // Evaluate h(x) from the measurement model y = h(x) + v
    // For gyroscope: h(x) = ω (angular velocity from state)
    Eigen::VectorXd MeasurementGyroscope::predict(const Eigen::VectorXd& x,
                                                  const system::SystemEstimator& system) const {
        // Extract angular velocity from state
        Eigen::VectorXd h(3);
        h = x.segment<3>(3);
        return h;
    }

    // Evaluate h(x) and its Jacobian J = dh/dx
    Eigen::VectorXd MeasurementGyroscope::predict(const Eigen::VectorXd& x,
                                                  const system::SystemEstimator& system,
                                                  Eigen::MatrixXd& dhdx) const {
        Eigen::VectorXd h = predict(x, system);

        // Jacobian for gyroscope measurement
        dhdx.resize(3, x.size());
        dhdx.setZero();

        // Set identity block where angular velocity is in state
        dhdx(0, 3) = 1.0;  // dωx/dωx = 1
        dhdx(1, 4) = 1.0;  // dωy/dωy = 1
        dhdx(2, 5) = 1.0;  // dωz/dωz = 1

        return h;
    }

    // Evaluate h(x), Jacobian, and Hessian
    Eigen::VectorXd MeasurementGyroscope::predict(const Eigen::VectorXd& x,
                                                  const system::SystemEstimator& system,
                                                  Eigen::MatrixXd& dhdx,
                                                  Eigen::Tensor<double, 3>& d2hdx2) const {
        Eigen::VectorXd h = predict(x, system, dhdx);

        // Hessian for gyroscope measurement
        // Since h(x) is linear (identity mapping), all second derivatives are zero
        d2hdx2.resize(3, x.size(), x.size());
        d2hdx2.setZero();

        return h;
    }

    // Define measurement noise covariance
    gaussian::GaussianInfo<double> MeasurementGyroscope::noiseDensity(
        const system::SystemEstimator& system) const {

        // SR is an upper triangular matrix such that SR^T * SR = R
        // where R is the measurement noise covariance
        Eigen::MatrixXd SR = sigma_gyro_ * Eigen::MatrixXd::Identity(3, 3);

        return gaussian::GaussianInfo<double>::fromSqrtMoment(SR);
    }

} // namespace utility::slam::measurement
