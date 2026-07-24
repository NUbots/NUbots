/**
 * @file MeasurementFieldLines.h
 * @brief Field-line point measurement against the analytic field line map.
 */
#ifndef MEASUREMENTFIELDLINES_HPP
#define MEASUREMENTFIELDLINES_HPP

#include <Eigen/Core>
#include <vector>

#include "../FieldMap.hpp"
#include "../FieldSamples.hpp"
#include "../camera/Pose.hpp"
#include "../rotation.hpp"
#include "../system/SystemEstimator.hpp"
#include "../system/SystemLocalisation.hpp"
#include "Measurement.hpp"

/**
 * @class MeasurementFieldLines
 * @brief Dense field-line point measurement (map-matching likelihood).
 *
 * Each field-line detection is a unit ray in the camera frame {c}. Given the
 * state, the ray is intersected with the field ground plane and the resulting
 * point compared against the nearest painted line of the field map:
 *
 *   log p = sum_j logsumexp( log(w_in) + logN(d_j; 0, sigma_j^2),
 *                            log(1 - w_in) + log p_clutter )
 *
 * where d_j is the distance to the nearest line and sigma_j grows with the
 * projection range (grazing rays are noisier). This is the map-matching
 * approach of Lauer et al. ("Calculating the Perfect Match", RoboCup 2006)
 * and the NUbots field-line distance map, expressed as a proper likelihood
 * inside the MAP/Laplace framework.
 *
 * Rays whose ground intersection is invalid (upward or grazing) at the prior
 * mean are excluded once at construction so the cost surface keeps a fixed
 * term count during optimisation.
 */

namespace utility::slam::measurement {

    using utility::slam::camera::Pose;
    using utility::slam::gaussian::GaussianInfo;
    using utility::slam::system::SystemLocalisation;

    class MeasurementFieldLines : public Measurement {
    public:
        /**
         * @brief Noise and selection options.
         */
        /**
         * Defaults calibrated against the recorded data: at NUbots' own converged
         * pose the line points sit a median 0.17-0.38 m from the line map with
         * 25-55% beyond 0.3 m, and per-frame errors are correlated through the
         * shared camera pose, so the point budget is kept small and the noise
         * model deliberately loose.
         */
        struct Options {
            double sigmaDistance     = 0.25;  ///< Base line-distance noise std dev [m]
            double sigmaAngular      = 0.02;  ///< Ray angular noise mapped to ground range [rad]
            double inlierProbability = 0.55;  ///< Mixture weight of the inlier component
            double clutterArea       = 40.0;  ///< Clutter uniform density area [m^2]
            std::size_t maxPoints    = 20;    ///< Subsample cap on line points per update
            double maxRange          = 2.5;   ///< Maximum ground projection range at the prior [m]
            double minDownward       = 0.05;  ///< Minimum downward ray component at the prior
        };

        /**
         * @brief Construct a field-line measurement.
         * @param time Event time [s]
         * @param sample Field-line points as unit rays in {c}
         * @param Tbc Camera pose w.r.t. torso (kinematics at capture time)
         * @param map Field map providing the analytic line distance
         * @param system System whose prior mean selects the usable rays
         * @param options Noise and selection options
         */
        MeasurementFieldLines(double time,
                              const LinePointsSample& sample,
                              const Pose<double>& Tbc,
                              const FieldMap& map,
                              const SystemLocalisation& system,
                              const Options& options);

        /**
         * @brief Construct with default options.
         */
        MeasurementFieldLines(double time,
                              const LinePointsSample& sample,
                              const Pose<double>& Tbc,
                              const FieldMap& map,
                              const SystemLocalisation& system);

        virtual Eigen::VectorXd simulate(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& system) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g) const override;
        virtual double logLikelihood(const Eigen::VectorXd& x,
                                     const SystemEstimator& system,
                                     Eigen::VectorXd& g,
                                     Eigen::MatrixXd& H) const override;

        /**
         * @brief Templated log-likelihood for autodiff.
         */
        template <typename Scalar>
        Scalar logLikelihoodImpl(const Eigen::VectorX<Scalar>& x) const;

        std::size_t numPoints() const {
            return static_cast<std::size_t>(rays_.cols());
        }

    protected:
        virtual void update(SystemBase& system) override;

        const FieldMap& map_;                            ///< Field map (line primitives)
        Pose<double> Tbc_;                               ///< Camera pose w.r.t. torso at capture time
        Eigen::Matrix<double, 3, Eigen::Dynamic> rays_;  ///< Selected unit rays in {c}
        Eigen::VectorXd sigma2_;                         ///< Per-ray distance noise variance [m^2]
        Options options_;
    };

    template <typename Scalar>
    Scalar MeasurementFieldLines::logLikelihoodImpl(const Eigen::VectorX<Scalar>& x) const {
        using std::exp, std::log;

        const Eigen::Index n = rays_.cols();
        if (n == 0) {
            return Scalar(0);
        }

        // Camera pose in field frame with mount-bias correction:
        // Tfc = Tfb(x) * Tbc * R(deltaC)
        Pose<Scalar> Tbias(SystemLocalisation::cameraBiasRotation(x), Eigen::Vector3<Scalar>::Zero());
        Pose<Scalar> Tfc                  = SystemLocalisation::fieldPose(x) * Pose<Scalar>(Tbc_) * Tbias;
        const Eigen::Matrix3<Scalar> Rfc  = Tfc.rotationMatrix;
        const Eigen::Vector3<Scalar> rCFf = Tfc.translationVector;

        const double logInlierWeight = std::log(options_.inlierProbability);
        const double logClutter      = std::log(1.0 - options_.inlierProbability) - std::log(options_.clutterArea);

        Scalar logLik = Scalar(0);
        for (Eigen::Index j = 0; j < n; ++j) {
            // Intersect the ray with the field ground plane z = 0
            Eigen::Vector3<Scalar> d = Rfc * rays_.col(j).cast<Scalar>();
            Scalar dz                = d.z();
            if (dz > Scalar(-1e-3)) {
                dz = Scalar(-1e-3);  // Clamp: selection at the prior keeps this rare
            }
            Scalar lambda = -rCFf.z() / dz;
            Eigen::Vector2<Scalar> p(rCFf.x() + lambda * d.x(), rCFf.y() + lambda * d.y());

            Scalar d2 = map_.distanceSquaredToNearestLine(p);

            const double s2 = sigma2_(j);
            Scalar a        = Scalar(logInlierWeight - 0.5 * std::log(2.0 * M_PI * s2)) - Scalar(0.5) * d2 / Scalar(s2);
            Scalar b        = Scalar(logClutter);
            if (a > b) {
                logLik += a + log(Scalar(1) + exp(b - a));
            }
            else {
                logLik += b + log(Scalar(1) + exp(a - b));
            }
        }
        return logLik;
    }
}  // namespace utility::slam::measurement

#endif
