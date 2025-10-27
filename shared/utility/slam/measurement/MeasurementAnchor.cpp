#include "MeasurementAnchor.hpp"
#include <cmath>

namespace utility::slam::measurement {

    MeasurementAnchor::MeasurementAnchor(double time, const Eigen::VectorXd& y)
        : MeasurementGaussianLikelihood(time, y) {
        updateMethod_ = UpdateMethod::AFFINE;
    }

    MeasurementAnchor::MeasurementAnchor(double time, const Eigen::VectorXd& y, int verbosity)
        : MeasurementGaussianLikelihood(time, y, verbosity) {
        updateMethod_ = UpdateMethod::AFFINE;
    }

    MeasurementAnchor::~MeasurementAnchor() = default;

    std::string MeasurementAnchor::getProcessString() const {
        return "Anchor measurement update:";
    }

    // Evaluate h(x) from the measurement model y = h(x) + v
    // For anchor: h(x) = -e3^T*rCNn (anchor position from state)
    Eigen::VectorXd MeasurementAnchor::predict(const Eigen::VectorXd& x,
                                                  const system::SystemEstimator& system) const {
        // Extract anchor position from state
        Eigen::VectorXd rBNn = x.segment<3>(6);
        Eigen::VectorXd rCNn = rBNn;
        Eigen::VectorXd h(1);
        h(0) = rCNn(2);
        return h;
    }

    // Evaluate h(x) and its Jacobian J = dh/dx
    Eigen::VectorXd MeasurementAnchor::predict(const Eigen::VectorXd& x,
                                                  const system::SystemEstimator& system,
                                                  Eigen::MatrixXd& dhdx) const {
        Eigen::VectorXd h = predict(x, system);

        // Jacobian for anchor measurement
        dhdx.resize(1, x.size());
        dhdx.setZero();

        dhdx(0, 8) = 1.0;  // down component of state
        return h;
    }

    // Evaluate h(x), Jacobian, and Hessian
    Eigen::VectorXd MeasurementAnchor::predict(const Eigen::VectorXd& x,
                                                  const system::SystemEstimator& system,
                                                  Eigen::MatrixXd& dhdx,
                                                  Eigen::Tensor<double, 3>& d2hdx2) const {
        Eigen::VectorXd h = predict(x, system, dhdx);

        // Hessian for anchor measurement
        // Since h(x) is linear (identity mapping), all second derivatives are zero
        d2hdx2.resize(1, x.size(), x.size());
        d2hdx2.setZero();

        return h;
    }

    // Define measurement noise covariance
    gaussian::GaussianInfo<double> MeasurementAnchor::noiseDensity(
        const system::SystemEstimator& system) const {

        // SR is an upper triangular matrix such that SR^T * SR = R
        // where R is the measurement noise covariance
        Eigen::MatrixXd SR = sigma_anchor_ * Eigen::MatrixXd::Identity(1, 1);

        return gaussian::GaussianInfo<double>::fromSqrtMoment(SR);
    }

} // namespace utility::slam::measurement
