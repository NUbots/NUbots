#include "MeasurementQuaternionNorm.hpp"

#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>

#include "utility/gaussian_filtering/measurement/Measurement.hpp"

namespace module::localisation::measurement {

    double MeasurementQuaternionNorm::logLikelihood(const Eigen::VectorXd& x,
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

    double MeasurementQuaternionNorm::logLikelihood(const Eigen::VectorXd& x,
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
