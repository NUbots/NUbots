/**
 * @file MeasurementFieldLandmarks.h
 * @brief Vision measurement of known field landmarks from YOLO detections.
 */
#ifndef MEASUREMENTFIELDLANDMARKS_HPP
#define MEASUREMENTFIELDLANDMARKS_HPP

#include <Eigen/Core>
#include <string>
#include <vector>

#include "../FieldMap.hpp"
#include "../FieldSamples.hpp"
#include "../camera/Pose.hpp"
#include "../rotation.hpp"
#include "../system/SystemEstimator.hpp"
#include "../system/SystemLocalisation.hpp"
#include "Measurement.hpp"

/**
 * @class MeasurementFieldLandmarks
 * @brief Measurement event for YOLO field landmark detections (goal posts, L/T/X intersections).
 *
 * Each detection provides a unit ray in the camera frame {c}:
 *  - intersections: ray to the bounding box centre,
 *  - goal posts: ray to the bottom-centre of the bounding box (post base on the ground).
 *
 * Detections are associated to mapped landmarks of the same class by greedy
 * surprisal-nearest-neighbour (SNN) assignment against rays predicted at the
 * prior mean, inside a geometric pre-gate (see associate()).
 *
 * The likelihood for each associated pair is an isotropic Gaussian on the
 * chordal residual e = u_meas - u_pred, which agrees with an angular error
 * Gaussian to second order, mixed with uniform clutter over the sphere at a
 * per-detection weight w_j taken from the YOLO confidence:
 *   log p = sum_j log[ w_j N(e_j; sigma^2 I) + (1 - w_j)/(4 pi) ]
 *
 * The MAP update through Measurement::update yields the Laplace-approximation
 * posterior (mean and sqrt information) in the usual way.
 */

namespace utility::slam::measurement {

    using utility::slam::camera::Pose;
    using utility::slam::gaussian::GaussianInfo;
    using utility::slam::system::SystemLocalisation;

    class MeasurementFieldLandmarks : public Measurement {
    public:
        /**
         * @brief Association and noise options.
         *
         * Defaults calibrated against the recorded NUbots data: ground-projected
         * intersection detections scatter 0.3-1.0 m at 3-5 m range (5-10 deg angular)
         * with occasional gross outliers, so the likelihood is an inlier Gaussian
         * mixed with a uniform clutter component over the unit sphere.
         */
        struct Options {
            double sigmaAngular =
                0.25;  ///< Inlier ray angular noise std dev [rad] (total per-frame error incl. systematic)
            double gateAngle         = 0.35;  ///< Max association residual angle [rad] (~20 deg)
            double minConfidence     = 0.5;   ///< Reject detections below this confidence outright
            double inlierProbability = 0.7;   ///< Inlier mixture weight at confidenceReference

            // YOLO confidence is (roughly) the probability that a box is a true
            // positive, which is exactly what the inlier weight of the robust
            // mixture means -- so confidence scales that weight rather than the
            // noise sigma. Downweighting via sigma would claim the landmark is
            // certainly real but poorly measured; the actual failure mode of a
            // weak detection is that it is not a landmark at all. As w -> 0 the
            // per-detection likelihood tends to the flat clutter term, which
            // contributes almost nothing to the gradient AND almost nothing to the
            // Hessian, so a weak detection cannot sharpen the posterior: admitting
            // them is safe against overconfidence in a way that simply lowering
            // minConfidence under a fixed weight would not be.
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
        std::size_t numCandidates() const {
            return candidates_.size();
        }
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& measuredRays() const {
            return uMeas_;
        }
        const Eigen::Matrix<double, 3, Eigen::Dynamic>& associatedLandmarks() const {
            return rLFf_;
        }

        /**
         * @brief What the final association pass did with one usable detection.
         *
         * Only detections of a mapped class that cleared the confidence threshold
         * appear here; everything else never reached association at all.
         */
        struct DetectionOutcome {
            std::size_t detection;    ///< Index into the vision sample's detection list
            bool associated = false;  ///< Matched a map landmark within the gate and won the assignment
        };

        /// @brief Association outcome of every usable detection (for the visualiser).
        std::vector<DetectionOutcome> detectionOutcomes() const;

        /**
         * @brief Re-associate the detections against the system's current pose.
         *
         * Used by the hypothesis bank so each mixture component is scored against
         * the landmark assignment implied by its OWN pose. The 180 deg field mirror
         * is the case that matters: re-associated at its own pose it fits the
         * mirror-partner landmarks exactly as well as the true pose fits the
         * originals, so on-field evidence leaves the two equally weighted (the
         * asymmetry that separates them comes from the out-of-field map instead).
         */
        void reassociate(const SystemEstimator& system) override {
            assocKeys_ = associate(system.density.mean(), system.density.cov());
        }

    protected:
        /**
         * @brief MAP update with iterated re-association (cf. iterative landmark matching).
         *
         * After the MAP optimisation, detections are re-associated at the posterior
         * mean; if the association set changed, the prior is restored and the
         * optimisation re-run, for at most maxAssociationIterations_ passes.
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
         * Surprisal nearest neighbour (SNN): pairs are ranked by how much better the
         * inlier component explains them than the uniform clutter component, i.e. by
         * the surprisal (negative log predictive density, evaluated in the tangent
         * plane of the predicted bearing) minus the clutter-crossover surprisal.
         * Ranking on raw angle instead would ignore two things that matter here: a
         * near landmark's predicted bearing is far more sensitive to pose error than
         * a distant one's, and a weak detection is far more likely to be spurious --
         * so a plain nearest-angle winner can be a low-confidence box that steals a
         * landmark from a confident detection and leaves it unassociated.
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
        std::vector<std::size_t> assocCand_;                          ///< Candidate index behind each column of uMeas_
        Options options_;
        int maxAssociationIterations_ = 1;  ///< Maximum association/optimisation passes
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
            // Robust mixture: inlier Gaussian on the chordal residual + uniform
            // clutter over the unit sphere (density 1/(4 pi) per steradian). The
            // inlier weight is per-detection, set from the YOLO confidence.
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
}  // namespace utility::slam::measurement

#endif
