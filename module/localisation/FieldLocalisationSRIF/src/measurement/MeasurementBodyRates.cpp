#include "MeasurementBodyRates.hpp"

#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

#include "utility/gaussian_filtering/measurement/Measurement.hpp"

// Both models are linear in the state, so the exact Hessian is constant and the
// trust-region Newton update converges in one step.

namespace module::localisation::measurement {

    using srif::SystemLocalisation;

    // =====================================================================
    // MeasurementGyroscope
    // =====================================================================

    MeasurementGyroscope::MeasurementGyroscope(double time, const Eigen::Vector3d& gyroscope, double sigma)
        : Measurement(time), y_(gyroscope), sigma_(sigma) {
        updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    Eigen::VectorXd MeasurementGyroscope::simulate(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return x.segment<3>(SystemLocalisation::iOmega) + x.segment<3>(SystemLocalisation::iGyroBias);
    }

    double MeasurementGyroscope::logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return logLikelihoodImpl<double>(x);
    }

    double MeasurementGyroscope::logLikelihood(const Eigen::VectorXd& x,
                                               const SystemEstimator& /*system*/,
                                               Eigen::VectorXd& g) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::gradient;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        dual fdual;
        auto func = [this](const Eigen::VectorX<dual>& xd) -> dual {
            return this->template logLikelihoodImpl<dual>(xd);
        };
        g = gradient(func, wrt(xdual), at(xdual), fdual);
        return static_cast<double>(fdual);
    }

    double MeasurementGyroscope::logLikelihood(const Eigen::VectorXd& x,
                                               const SystemEstimator& /*system*/,
                                               Eigen::VectorXd& g,
                                               Eigen::MatrixXd& H) const {
        using autodiff::at;
        using autodiff::dual2nd;
        using autodiff::hessian;
        using autodiff::wrt;

        g.resize(x.size());
        H.resize(x.size(), x.size());

        Eigen::VectorX<dual2nd> xdual = x.cast<dual2nd>();
        dual2nd fdual;
        auto func = [this](const Eigen::VectorX<dual2nd>& xd) -> dual2nd {
            return this->template logLikelihoodImpl<dual2nd>(xd);
        };
        H = hessian(func, wrt(xdual), at(xdual), fdual, g);
        return static_cast<double>(fdual);
    }

    // =====================================================================
    // MeasurementBodyVelocity
    // =====================================================================

    MeasurementBodyVelocity::MeasurementBodyVelocity(double time, const Eigen::Vector3d& velocity, double sigma)
        : Measurement(time), y_(velocity), sigma_(sigma) {
        updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    Eigen::VectorXd MeasurementBodyVelocity::simulate(const Eigen::VectorXd& x,
                                                      const SystemEstimator& /*system*/) const {
        return x.segment<3>(SystemLocalisation::iVel);
    }

    double MeasurementBodyVelocity::logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return logLikelihoodImpl<double>(x);
    }

    double MeasurementBodyVelocity::logLikelihood(const Eigen::VectorXd& x,
                                                  const SystemEstimator& /*system*/,
                                                  Eigen::VectorXd& g) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::gradient;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        dual fdual;
        auto func = [this](const Eigen::VectorX<dual>& xd) -> dual {
            return this->template logLikelihoodImpl<dual>(xd);
        };
        g = gradient(func, wrt(xdual), at(xdual), fdual);
        return static_cast<double>(fdual);
    }

    double MeasurementBodyVelocity::logLikelihood(const Eigen::VectorXd& x,
                                                  const SystemEstimator& /*system*/,
                                                  Eigen::VectorXd& g,
                                                  Eigen::MatrixXd& H) const {
        using autodiff::at;
        using autodiff::dual2nd;
        using autodiff::hessian;
        using autodiff::wrt;

        g.resize(x.size());
        H.resize(x.size(), x.size());

        Eigen::VectorX<dual2nd> xdual = x.cast<dual2nd>();
        dual2nd fdual;
        auto func = [this](const Eigen::VectorX<dual2nd>& xd) -> dual2nd {
            return this->template logLikelihoodImpl<dual2nd>(xd);
        };
        H = hessian(func, wrt(xdual), at(xdual), fdual, g);
        return static_cast<double>(fdual);
    }

}  // namespace module::localisation::measurement
