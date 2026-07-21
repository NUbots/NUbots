#include "MeasurementKinematicHeight.hpp"

#include <Eigen/Core>
#include <cmath>

#include "Measurement.hpp"

namespace utility::slam::measurement {
    MeasurementKinematicHeight::MeasurementKinematicHeight(double time, double height, double sigma)
        : Measurement(time), y_(height), sigma_(sigma) {
        // Linear-Gaussian in the state: closed-form quadratic log-likelihood
        updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    Eigen::VectorXd MeasurementKinematicHeight::simulate(const Eigen::VectorXd& x,
                                                         const SystemEstimator& /*system*/) const {
        Eigen::VectorXd y(1);
        y << x(2);
        return y;
    }

    double MeasurementKinematicHeight::logLikelihood(const Eigen::VectorXd& x,
                                                     const SystemEstimator& /*system*/) const {
        const double sigma2 = sigma_ * sigma_;
        const double e      = y_ - x(2);
        return -0.5 * std::log(2.0 * M_PI * sigma2) - 0.5 * e * e / sigma2;
    }

    double MeasurementKinematicHeight::logLikelihood(const Eigen::VectorXd& x,
                                                     const SystemEstimator& system,
                                                     Eigen::VectorXd& g) const {
        const double sigma2 = sigma_ * sigma_;
        g                   = Eigen::VectorXd::Zero(x.size());
        g(2)                = (y_ - x(2)) / sigma2;
        return logLikelihood(x, system);
    }

    double MeasurementKinematicHeight::logLikelihood(const Eigen::VectorXd& x,
                                                     const SystemEstimator& system,
                                                     Eigen::VectorXd& g,
                                                     Eigen::MatrixXd& H) const {
        const double sigma2 = sigma_ * sigma_;
        H                   = Eigen::MatrixXd::Zero(x.size(), x.size());
        H(2, 2)             = -1.0 / sigma2;
        return logLikelihood(x, system, g);
    }

}  // namespace utility::slam::measurement
