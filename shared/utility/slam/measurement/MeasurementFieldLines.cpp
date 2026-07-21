#include "MeasurementFieldLines.hpp"

#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <cassert>
#include <cmath>
#include <vector>

#include "../FieldMap.hpp"
#include "../system/SystemLocalisation.hpp"
#include "Measurement.hpp"

namespace utility::slam::measurement {

    MeasurementFieldLines::MeasurementFieldLines(double time,
                                                 const LinePointsSample& sample,
                                                 const Pose<double>& Tbc,
                                                 const FieldMap& map,
                                                 const SystemLocalisation& system,
                                                 const Options& options)
        : Measurement(time), map_(map), Tbc_(Tbc), options_(options) {
        updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;

        // Select usable rays at the prior mean: downward-looking with a bounded
        // ground projection range, subsampled to the point budget. Fixing the
        // selection here keeps the term count of the cost constant during the
        // optimisation.
        const Eigen::VectorXd xPrior = system.density.mean();
        Pose<double> Tbias(SystemLocalisation::cameraBiasRotation<double>(xPrior), Eigen::Vector3d::Zero());
        Pose<double> Tfc            = SystemLocalisation::fieldPose<double>(xPrior) * Tbc_ * Tbias;
        const Eigen::Matrix3d& Rfc  = Tfc.rotationMatrix;
        const Eigen::Vector3d& rCFf = Tfc.translationVector;

        const Eigen::Index nAll = sample.rays.cols();
        std::vector<Eigen::Index> usable;
        std::vector<double> range;
        usable.reserve(nAll);
        for (Eigen::Index j = 0; j < nAll; ++j) {
            Eigen::Vector3d d = Rfc * sample.rays.col(j);
            if (d.z() >= -options_.minDownward) {
                continue;  // Upward or grazing ray
            }
            double lambda = -rCFf.z() / d.z();
            if (lambda <= 0 || lambda > options_.maxRange) {
                continue;
            }
            usable.push_back(j);
            range.push_back(lambda);
        }

        const std::size_t stride =
            std::max<std::size_t>(1, (usable.size() + options_.maxPoints - 1) / options_.maxPoints);
        std::vector<Eigen::Index> selected;
        std::vector<double> selectedRange;
        for (std::size_t k = 0; k < usable.size(); k += stride) {
            selected.push_back(usable[k]);
            selectedRange.push_back(range[k]);
        }

        rays_.resize(3, static_cast<Eigen::Index>(selected.size()));
        sigma2_.resize(static_cast<Eigen::Index>(selected.size()));
        for (std::size_t k = 0; k < selected.size(); ++k) {
            rays_.col(static_cast<Eigen::Index>(k)) = sample.rays.col(selected[k]);
            // Angular noise maps to ground distance noise ~ range * sigma_ang / sin(elevation)
            Eigen::Vector3d d         = Rfc * sample.rays.col(selected[k]);
            const double sinElevation = std::max(0.05, -d.z());
            const double sigmaRange   = selectedRange[k] * options_.sigmaAngular / sinElevation;
            sigma2_(static_cast<Eigen::Index>(k)) =
                options_.sigmaDistance * options_.sigmaDistance + sigmaRange * sigmaRange;
        }
    }

    MeasurementFieldLines::MeasurementFieldLines(double time,
                                                 const LinePointsSample& sample,
                                                 const Pose<double>& Tbc,
                                                 const FieldMap& map,
                                                 const SystemLocalisation& system)
        : MeasurementFieldLines(time, sample, Tbc, map, system, Options{}) {}

    Eigen::VectorXd MeasurementFieldLines::simulate(const Eigen::VectorXd& /*x*/,
                                                    const SystemEstimator& /*system*/) const {
        // The "noise-free measurement" is zero distance to the nearest line for every point
        return Eigen::VectorXd::Zero(rays_.cols());
    }

    double MeasurementFieldLines::logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return logLikelihoodImpl<double>(x);
    }

    double MeasurementFieldLines::logLikelihood(const Eigen::VectorXd& x,
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

    double MeasurementFieldLines::logLikelihood(const Eigen::VectorXd& x,
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

    void MeasurementFieldLines::update(SystemBase& system) {
        if (rays_.cols() == 0) {
            return;  // No usable points; leave the density untouched
        }
        Measurement::update(system);
    }
}  // namespace utility::slam::measurement
