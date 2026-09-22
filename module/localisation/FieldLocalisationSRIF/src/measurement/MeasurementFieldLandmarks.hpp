/**
 * @file MeasurementFieldLandmarks.hpp
 * @brief Vision measurement of known field landmarks from YOLO detections.
 */
#ifndef MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLANDMARKS_HPP
#define MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLANDMARKS_HPP

#include <Eigen/Core>
#include <string>
#include <vector>

#include "srif/FieldMap.hpp"
#include "srif/FieldSamples.hpp"
#include "srif/SystemLocalisation.hpp"

#include "utility/gaussian_filtering/Pose.hpp"
#include "utility/gaussian_filtering/measurement/Measurement.hpp"
#include "utility/gaussian_filtering/rotation.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"

/**
 * @class MeasurementFieldLandmarks
 * @brief Measurement event for YOLO field landmark detections (goal posts, L/T/X intersections).
 *
 * Each detection gives a unit ray in the camera frame {c}: to the bounding box
 * centre for intersections, to its bottom-centre for goal posts. Rays are
 * associated to mapped landmarks of the same class by greedy surprisal-nearest-
 * neighbour assignment (see associate()).
 *
 * Each associated pair contributes an isotropic Gaussian on the chordal residual
 * e = u_meas - u_pred mixed with uniform clutter over the sphere, weighted by the
 * detection's YOLO confidence:
 *   log p = sum_j log[ w_j N(e_j; sigma^2 I) + (1 - w_j)/(4 pi) ]
 */

namespace module::localisation::measurement {

    using srif::Detection;
    using srif::FieldMap;
    using srif::LandmarkType;
    using srif::SystemLocalisation;
    using srif::VisionSample;
    using utility::gaussian_filtering::Pose;
    using utility::gaussian_filtering::tangentBasis;
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemBase;
    using utility::gaussian_filtering::system::SystemEstimator;

    class MeasurementFieldLandmarks : public Measurement {
    public:
        /**
         * @brief Association and noise options.
         */
        struct Options {
            double sigmaAngular =
                0.25;  ///< Inlier ray angular noise std dev [rad] (total per-frame error incl. systematic)
            double gateAngle = 0.35;  ///< Max association residual angle [rad] (~20 deg)

            /// Yaw std devs the pre-gate widens by, so an uncertain belief can re-associate
            double gateYawScale = 2.0;
            double gateAngleMax = 1.0;  ///< Ceiling on the widened pre-gate [rad] (~57 deg)

            double minConfidence     = 0.5;  ///< Reject detections below this confidence outright
            double inlierProbability = 0.7;  ///< Inlier mixture weight at confidenceReference

            double confidenceReference  = 0.7;   ///< Confidence that maps to inlierProbability
            double maxInlierProbability = 0.95;  ///< Cap: no detection is ever treated as certain
        };

        /**
         * @brief Construct and associate a landmark measurement.
         * @param time Event time [s]
         * @param sample Vision sample containing YOLO detections (rays in {c})
         * @param Tbc Camera pose w.r.t. torso (from kinematics; Tbc = Htw * Hcw^{-1} at capture time)
         * @param map Field landmark map
         * @param system System whose prior mean is used for data association
         * @param options Association and noise options
         */
        MeasurementFieldLandmarks(double time,
                                  const VisionSample& sample,
                                  const Pose<double>& Tbc,
                                  const FieldMap& map,
                                  const SystemLocalisation& system,
                                  const Options& options);

        /**
         * @brief Construct with default options.
         */
        MeasurementFieldLandmarks(double time,
                                  const VisionSample& sample,
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

        /**
         * @brief Predicted unit rays in {c} for the associated landmarks.
         */
        template <typename Scalar>
        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> predictRays(const Eigen::VectorX<Scalar>& x) const;

        std::size_t numAssociated() const {
            return static_cast<std::size_t>(uMeas_.cols());
        }
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& measuredRays() const {
            return uMeas_;
        }

        /**
         * @brief Re-associate the detections against the system's current pose.
         *
         * Used by the hypothesis bank so each mixture component is scored against the
         * landmark assignment implied by its own pose rather than a shared one.
         */
        void reassociate(const SystemEstimator& system) override {
            assocKeys_ = associate(system.density.mean(), system.density.cov());
        }

    protected:
        /**
         * @brief MAP update, re-associating at the posterior mean and re-running the
         * optimisation from the prior if the association set changed.
         */
        virtual void update(SystemBase& system) override;

        /**
         * @brief Extract the measurement ray and landmark class for a detection.
         * @return true if the detection is a usable landmark class
         */
        static bool detectionRay(const Detection& det, Eigen::Vector3d& ray, LandmarkType& type);

        /**
         * @brief A usable detection before association.
         */
        struct CandidateDetection {
            Eigen::Vector3d ray;
            LandmarkType type;
            std::size_t detection;  ///< Index into the vision sample's detection list (for display)
            double inlierWeight;    ///< Mixture inlier weight implied by this detection's confidence
        };

        /**
         * @brief (Re)build the associated pairs from the stored candidates at the given state.
         *
         * Surprisal nearest neighbour: pairs are ranked by how much better the inlier
         * component explains them than the uniform clutter component, so ranking
         * accounts for bearing sensitivity and detection confidence as well as angle.
         *
         * @param x State to predict the landmarks at
         * @param P State covariance, inflating each predicted bearing by the pose uncertainty
         * @return Association keys (candidate index, landmark index) for change detection
         */
        std::vector<std::pair<std::size_t, std::size_t>> associate(const Eigen::VectorXd& x, const Eigen::MatrixXd& P);

        const FieldMap& map_;                             ///< Field landmark map
        std::vector<CandidateDetection> candidates_;      ///< Usable detections (rays in {c})
        Pose<double> Tbc_;                                ///< Camera pose w.r.t. torso at capture time
        Eigen::Matrix<double, 3, Eigen::Dynamic> uMeas_;  ///< Measured unit rays in {c} (associated only)
        Eigen::Matrix<double, 3, Eigen::Dynamic> rLFf_;   ///< Associated landmark positions in {f}
        std::vector<double> inlierWeight_;                ///< Per-column mixture inlier weight (from confidence)
        std::vector<std::pair<std::size_t, std::size_t>> assocKeys_;  ///< Last association (candidate, landmark)
        Options options_;                                             ///< Association and noise options
        int maxAssociationIterations_ = 1;                            ///< Maximum association/optimisation passes
    };

    template <typename Scalar>
    Eigen::Matrix<Scalar, 3, Eigen::Dynamic> MeasurementFieldLandmarks::predictRays(
        const Eigen::VectorX<Scalar>& x) const {
        // Camera pose in field frame with mount-bias correction:
        // Tfc = Tfb(x) * Tbc * R(deltaC)
        Pose<Scalar> Tfb = SystemLocalisation::fieldPose(x);
        Pose<Scalar> Tbias(SystemLocalisation::cameraBiasRotation(x), Eigen::Vector3<Scalar>::Zero());
        Pose<Scalar> Tfc = Tfb * Pose<Scalar>(Tbc_) * Tbias;

        const Eigen::Matrix3<Scalar> Rcf  = Tfc.rotationMatrix.transpose();
        const Eigen::Vector3<Scalar> rCFf = Tfc.translationVector;

        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> uPred(3, rLFf_.cols());
        for (Eigen::Index j = 0; j < rLFf_.cols(); ++j) {
            Eigen::Vector3<Scalar> rLCc = Rcf * (rLFf_.col(j).cast<Scalar>() - rCFf);
            uPred.col(j)                = rLCc / rLCc.norm();
        }
        return uPred;
    }

    template <typename Scalar>
    Scalar MeasurementFieldLandmarks::logLikelihoodImpl(const Eigen::VectorX<Scalar>& x) const {
        const Eigen::Index n = uMeas_.cols();
        if (n == 0) {
            return Scalar(0);
        }

        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> uPred = predictRays<Scalar>(x);

        const double sigma2       = options_.sigmaAngular * options_.sigmaAngular;
        const double logNormConst = -std::log(2.0 * M_PI * sigma2);  // 2 effective DOF per ray

        using std::exp, std::log;

        Scalar logLik = Scalar(0);
        for (Eigen::Index j = 0; j < n; ++j) {
            // Inlier Gaussian on the chordal residual, mixed with uniform clutter
            // over the unit sphere at density 1/(4 pi) per steradian
            const double w               = inlierWeight_[static_cast<std::size_t>(j)];
            const double logInlierWeight = std::log(w);
            const double logClutter      = std::log(1.0 - w) - std::log(4.0 * M_PI);

            Eigen::Vector3<Scalar> e = uMeas_.col(j).cast<Scalar>() - uPred.col(j);
            Scalar a = Scalar(logInlierWeight + logNormConst) - Scalar(0.5) * e.squaredNorm() / Scalar(sigma2);
            Scalar b = Scalar(logClutter);
            // Stable log-sum-exp of the two mixture components
            if (a > b) {
                logLik += a + log(Scalar(1) + exp(b - a));
            }
            else {
                logLik += b + log(Scalar(1) + exp(a - b));
            }
        }
        return logLik;
    }
}  // namespace module::localisation::measurement

#endif  // MODULE_LOCALISATION_MEASUREMENT_MEASUREMENTFIELDLANDMARKS_HPP
