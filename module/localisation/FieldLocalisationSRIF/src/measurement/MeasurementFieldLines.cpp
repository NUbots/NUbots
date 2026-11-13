/*
 * MIT License
 *
 * Copyright (c) 2026 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "MeasurementFieldLines.hpp"

#include <Eigen/Core>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <cassert>
#include <cmath>
#include <vector>

#include "srif/FieldMap.hpp"
#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/measurement/Measurement.hpp"

namespace module::localisation::measurement {

    MeasurementFieldLines::MeasurementFieldLines(double time,
                                                 const LinePointsSample& sample,
                                                 const Pose<double>& Tbc,
                                                 const FieldMap& map,
                                                 const SystemLocalisation& system,
                                                 const Options& options)
        : Measurement(time), map_(map), Tbc_(Tbc), options_(options) {
        update_method_ = UpdateMethod::NEWTONTRUSTEIG;

        // Select usable rays at the prior mean: downward-looking with a bounded ground
        // projection range, subsampled to the point budget
        const Eigen::VectorXd x_prior = system.density.mean();
        Pose<double> Tbias(SystemLocalisation::camera_bias_rotation<double>(x_prior), Eigen::Vector3d::Zero());
        Pose<double> Tfc            = SystemLocalisation::field_pose<double>(x_prior) * Tbc_ * Tbias;
        const Eigen::Matrix3d& Rfc  = Tfc.rotation_matrix;
        const Eigen::Vector3d& rCFf = Tfc.translation_vector;

        const Eigen::Index n_all = sample.rays.cols();
        std::vector<Eigen::Index> usable;
        std::vector<double> range;
        usable.reserve(n_all);
        for (Eigen::Index j = 0; j < n_all; ++j) {
            Eigen::Vector3d d = Rfc * sample.rays.col(j);
            if (d.z() >= -options_.min_downward) {
                continue;  // Upward or grazing ray
            }
            double lambda = -rCFf.z() / d.z();
            if (lambda <= 0 || lambda > options_.max_range) {
                continue;
            }
            usable.push_back(j);
            range.push_back(lambda);
        }

        const std::size_t stride =
            std::max<std::size_t>(1, (usable.size() + options_.max_points - 1) / options_.max_points);
        std::vector<Eigen::Index> selected;
        std::vector<double> selected_range;
        for (std::size_t k = 0; k < usable.size(); k += stride) {
            selected.push_back(usable[k]);
            selected_range.push_back(range[k]);
        }

        rays_.resize(3, static_cast<Eigen::Index>(selected.size()));
        sigma2_.resize(static_cast<Eigen::Index>(selected.size()));
        for (std::size_t k = 0; k < selected.size(); ++k) {
            rays_.col(static_cast<Eigen::Index>(k)) = sample.rays.col(selected[k]);
            // Angular noise maps to ground distance noise ~ range * sigma_ang / sin(elevation)
            Eigen::Vector3d d          = Rfc * sample.rays.col(selected[k]);
            const double sin_elevation = std::max(0.05, -d.z());
            const double sigma_range   = selected_range[k] * options_.sigma_angular / sin_elevation;
            sigma2_(static_cast<Eigen::Index>(k)) =
                options_.sigma_distance * options_.sigma_distance + sigma_range * sigma_range;
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
        // The noise-free measurement is zero distance to the nearest line for every point
        return Eigen::VectorXd::Zero(rays_.cols());
    }

    double MeasurementFieldLines::log_likelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return log_likelihood_impl<double>(x);
    }

    double MeasurementFieldLines::log_likelihood(const Eigen::VectorXd& x,
                                                 const SystemEstimator& /*system*/,
                                                 Eigen::VectorXd& g) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::gradient;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        dual fdual;
        auto func = [this](const Eigen::VectorX<dual>& xd) -> dual {
            return this->template log_likelihood_impl<dual>(xd);
        };
        g = gradient(func, wrt(xdual), at(xdual), fdual);
        return static_cast<double>(fdual);
    }

    double MeasurementFieldLines::log_likelihood(const Eigen::VectorXd& x,
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
            return this->template log_likelihood_impl<dual2nd>(xd);
        };
        H = hessian(func, wrt(xdual), at(xdual), fdual, g);
        return static_cast<double>(fdual);
    }

    void MeasurementFieldLines::update(SystemBase& system) {
        if (rays_.cols() == 0) {
            return;  // No usable points
        }
        Measurement::update(system);
    }
}  // namespace module::localisation::measurement
