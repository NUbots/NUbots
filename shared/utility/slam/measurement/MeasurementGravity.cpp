#include "MeasurementGravity.hpp"

#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

#include "Measurement.hpp"

namespace utility::slam::measurement {

    MeasurementGravity::MeasurementGravity(double time, const Eigen::Vector3d& accelerometer, double sigma)
        : Measurement(time), y_(accelerometer), sigma_(sigma) {
        updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;
    }

    Eigen::VectorXd MeasurementGravity::simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const {
        const Eigen::Matrix3d Rfb = rpy2rot(Eigen::Vector3d(x.segment<3>(3)));
        return Rfb.transpose() * Eigen::Vector3d(0, 0, gravity_);
    }

    double MeasurementGravity::logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const {
        return logLikelihoodImpl<double>(x);
    }

    double MeasurementGravity::logLikelihood(const Eigen::VectorXd& x,
                                             const SystemEstimator& system,
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

    double MeasurementGravity::logLikelihood(const Eigen::VectorXd& x,
                                             const SystemEstimator& system,
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
}  // namespace utility::slam::measurement
