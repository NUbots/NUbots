/**
 * @file SystemLocalisation.h
 * @brief Defines the SystemLocalisation class for humanoid robot field localisation.
 */
#ifndef SYSTEMLOCALISATION_HPP
#define SYSTEMLOCALISATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include "../Event.hpp"
#include "../FieldSamples.hpp"
#include "../camera/Pose.hpp"
#include "../gaussian/GaussianInfo.hpp"
#include "../kinematics_helper.hpp"
#include "../measurement/Measurement.hpp"
#include "../rotation.hpp"
#include "SystemEstimator.hpp"


namespace utility::slam::system {

    using utility::slam::camera::Pose;
    using utility::slam::gaussian::GaussianInfo;
    using utility::slam::measurement::Measurement;

    /**
     * @brief A body-fixed twist sample derived from odometry.
     */
    struct BodyTwistSample {
        double t;                 ///< Sample time [s] (relative to log start)
        Eigen::Vector3d vBb;      ///< Body-fixed translational velocity [m/s]
        Eigen::Vector3d omegaBb;  ///< Body-fixed angular velocity [rad/s]
    };

    /*
     * State contains the 6-DOF torso pose in the field frame {f} plus a 2-DOF
     * camera-mount attitude bias:
     *
     *     [ rBFf    ] Torso position in field frame (3)
     * x = [ Thetafb ] Torso orientation RPY angles (3), Rfb = rpy2rot(Thetafb)
     *     [ deltaC  ] Camera mount attitude bias (roll, pitch) about the camera axes (2)
     *
     * Field frame {f}: origin at centre of field on the ground plane, z up,
     * consistent with the NUbots Hfw convention.
     *
     * The camera bias models a constant error in the kinematic torso-to-camera
     * chain (evident in the recorded data as ground-projection errors growing
     * with range^2, ~1 m at 4-5 m range, consistent with a 1.5-2 deg pitch
     * bias). It is a random-walk state with very small process noise, applied
     * on the camera side of the extrinsic transform by vision measurements:
     *   Tfc = Tfb(x) * Tbc * R(deltaC)
     *
     * Prediction is driven by a buffer of body-fixed twist samples: linear
     * velocity from finite-differencing the odometry stream (Htw), angular
     * velocity from the torso gyroscope (bias-calibrated on quiet samples; the
     * walk-engine odometry attitude slips badly while turning, the gyro does
     * not). The twist is treated as a known input with additive process noise:
     *
     *   deta/dt = JK(eta) * nu(t) + dw,   ddeltaC/dt = dw_c
     *
     * where JK(eta) = blkdiag(Rfb, TK(Theta)) transports the body twist to
     * field-frame pose rates.
     */
    class SystemLocalisation : public SystemEstimator {
    public:
        /**
         * @brief Process noise and input handling parameters.
         */
        struct Parameters {
            double sigmaPosXY = 0.08;    ///< Position process noise PSD, horizontal [m/sqrt(s)] (floor vs collapse in
                                         ///< weakly-observable directions)
            double sigmaPosZ    = 0.02;  ///< Position process noise PSD, vertical [m/sqrt(s)]
            double sigmaAtt     = 0.05;  ///< Roll/pitch process noise PSD [rad/sqrt(s)]
            double sigmaYaw     = 0.05;  ///< Yaw process noise PSD [rad/sqrt(s)]
            double sigmaCamBias = 3e-4;  ///< Camera mount bias process noise PSD [rad/sqrt(s)] (kept small so bias
                                         ///< cannot wander while stationary)
        };

        static constexpr Eigen::Index nx = 8;  ///< State dimension

        SystemLocalisation(const GaussianInfo<double>& density, const std::vector<BodyTwistSample>& twistBuffer);
        virtual SystemLocalisation* clone() const;

        virtual Eigen::VectorXd dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const override;
        virtual Eigen::VectorXd dynamics(double t,
                                         const Eigen::VectorXd& x,
                                         const Eigen::VectorXd& u,
                                         Eigen::MatrixXd& J) const override;
        virtual Eigen::VectorXd input(double t, const Eigen::VectorXd& x) const override;
        virtual GaussianInfo<double> processNoiseDensity(double dt) const override;
        virtual std::vector<Eigen::Index> processNoiseIndex() const override;

        /**
         * @brief Torso pose in field frame from a state vector.
         * @param x State vector (6)
         * @return Tfb with rotationMatrix Rfb and translationVector rBFf
         */
        template <typename Scalar>
        static Pose<Scalar> fieldPose(const Eigen::VectorX<Scalar>& x) {
            Pose<Scalar> Tfb;
            Tfb.rotationMatrix    = rpy2rot(Eigen::Vector3<Scalar>(x.template segment<3>(3)));
            Tfb.translationVector = x.template segment<3>(0);
            return Tfb;
        }

        /**
         * @brief Camera-mount attitude bias correction from a state vector.
         * @param x State vector (8)
         * @return Rotation applied on the camera side of the extrinsic: R(deltaC)
         */
        template <typename Scalar>
        static Eigen::Matrix3<Scalar> cameraBiasRotation(const Eigen::VectorX<Scalar>& x) {
            Eigen::Vector3<Scalar> rpy;
            rpy << x(6), x(7), Scalar(0);
            return rpy2rot(rpy);
        }

        /**
         * @brief Build a body-twist buffer from the odometry and gyroscope streams.
         *
         * For consecutive odometry samples, the relative pose
         * DeltaT = Twt(t1)^{-1} * Twt(t2) with Twt = Htw^{-1} yields
         * vBb = Delta r / dt, stamped at the interval midpoint; omegaBb is the
         * interval-mean gyroscope reading minus the quiet-sample gyro bias
         * (falling back to log(Delta R) / dt when the gyro is not finite).
         * Samples spanning gaps larger than maxGap are skipped.
         *
         * @param sensors Time-ordered odometry samples (absolute time [s])
         * @param t0 Time origin subtracted from sample times [s]
         * @param maxGap Maximum sample spacing to difference across [s]
         */
        static std::vector<BodyTwistSample> twistFromOdometry(const std::vector<SensorsSample>& sensors,
                                                              double t0,
                                                              double maxGap = 0.1);

        GaussianInfo<double> positionDensity() const;     ///< Marginal density of rBFf
        GaussianInfo<double> orientationDensity() const;  ///< Marginal density of Thetafb

        /**
         * @brief Reset the state density and system clock (initialisation / relocalisation).
         * @param density New state density
         * @param time New system time [s]
         */
        void resetTo(const GaussianInfo<double>& density, double time);

        Parameters params;

        // ---------------------------------------------------------------------
        // Hypothesis bank (multi-hypothesis field-symmetry handling)
        //
        // The RoboCup field has a 180 deg rotational symmetry about its centre:
        // the pose (rBFf, Thetafb) and its mirror produce identical landmark
        // observations, so a single Gaussian cannot represent the true belief
        // when the symmetry is unbroken. Following the B-Human multi-hypothesis
        // approach (Rofer et al.), the belief is a weighted Gaussian mixture; each
        // component is an independent 8-DOF pose density carried through the same
        // predict/update machinery, and the weights are updated from the Laplace
        // log-evidence each measurement reports (Measurement::logEvidence()).
        //
        // When no hypotheses are active (components_ empty) the estimator behaves
        // exactly as a single-Gaussian filter over `density`. Activating the bank
        // (initialiseHypotheses / spawnMirror) switches process() to iterate over
        // all components; `density` then always mirrors the maximum-weight
        // component so all existing single-Gaussian accessors keep working.
        // ---------------------------------------------------------------------

        /**
         * @brief Parameters governing hypothesis spawning, pruning and merging.
         */
        struct HypothesisParameters {
            double minWeight          = 0.02;  ///< Prune components below this normalised weight
            std::size_t maxComponents = 4;     ///< Cap on the number of live components
            double mergePosition      = 0.30;  ///< Merge gate on position separation [m]
            double mergeYaw           = 0.20;  ///< Merge gate on yaw separation [rad]
            double respawnPosStd      = 0.60;  ///< Respawn a mirror when the lone component's
                                               ///< horizontal position std exceeds this [m]
        };

        HypothesisParameters hyp;

        /**
         * @brief Activate the mixture with the current density and its 180 deg mirror.
         *
         * Seeds two equally weighted hypotheses (the current pose and its field
         * mirror) so a symmetry ambiguity present at initialisation can be resolved
         * by later asymmetric evidence. No-op inputs (single confident pose) still
         * benefit: the wrong mirror is down-weighted and pruned automatically.
         */
        void initialiseHypotheses();

        /**
         * @brief Add the 180 deg mirror of the maximum-weight component as a new,
         *        equally weighted hypothesis (used for symmetry-flip recovery).
         */
        void spawnMirror();

        /**
         * @brief Process an event across every active hypothesis.
         *
         * For each component the shared system clock is rewound and the event is
         * applied through the ordinary single-Gaussian path (predict + update),
         * so the verified machinery is reused unchanged. Measurement events
         * additionally accumulate their Laplace log-evidence into the component
         * weight. Afterwards the mixture is normalised, merged, pruned and (if it
         * has collapsed to a single uncertain component) a mirror is respawned,
         * and `density` is set to the maximum-weight component.
         *
         * With no active hypotheses this is exactly `event.process(*this)`.
         *
         * @param event The event to apply (typically a Measurement)
         */
        void process(Event& event);

        /**
         * @brief Number of live hypotheses (1 in single-hypothesis mode).
         */
        std::size_t numHypotheses() const {
            return components_.empty() ? 1 : components_.size();
        }

        /**
         * @brief Normalised (sum-to-one) linear weights of the live hypotheses.
         */
        std::vector<double> hypothesisWeights() const;

        /**
         * @brief Read-only access to the live hypothesis densities.
         */
        const std::vector<GaussianInfo<double>>& hypotheses() const {
            return components_;
        }

        /**
         * @brief The 180 deg field-symmetry mirror of a state vector.
         *
         * Rotates the pose by pi about the field-centre z axis:
         *   (x, y) -> (-x, -y),  yaw -> yaw + pi,  roll/pitch/z/cam-bias unchanged.
         */
        static Eigen::VectorXd mirrorState(const Eigen::VectorXd& x);

        /**
         * @brief The 180 deg field-symmetry mirror of a pose density.
         */
        static GaussianInfo<double> mirrorDensity(const GaussianInfo<double>& g);

    protected:
        const std::vector<BodyTwistSample>* twistBuffer_;  ///< Non-owning; ZOH input lookup

        std::vector<GaussianInfo<double>> components_;  ///< Mixture components (empty => single-hypothesis)
        std::vector<double> logWeights_;                ///< Unnormalised log weights per component

        void normaliseWeights();      ///< Renormalise logWeights_ (subtract log-sum-exp)
        void mergeComponents();       ///< Merge components within the merge gate (keep-best)
        void pruneComponents();       ///< Drop low-weight components; cap to maxComponents
        void respawnIfUnconfident();  ///< Respawn a mirror if collapsed to one uncertain component
        void setRepresentative();     ///< Set `density` to the maximum-weight component
    };

}  // namespace utility::slam::system

#endif
