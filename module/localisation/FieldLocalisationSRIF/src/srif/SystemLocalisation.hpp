/**
 * @file SystemLocalisation.hpp
 * @brief Defines the SystemLocalisation class for humanoid robot field localisation.
 */
#ifndef MODULE_LOCALISATION_SRIF_SYSTEMLOCALISATION_HPP
#define MODULE_LOCALISATION_SRIF_SYSTEMLOCALISATION_HPP

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

#include "srif/FieldSamples.hpp"

#include "utility/gaussian_filtering/Event.hpp"
#include "utility/gaussian_filtering/Pose.hpp"
#include "utility/gaussian_filtering/gaussian/GaussianInfo.hpp"
#include "utility/gaussian_filtering/measurement/Measurement.hpp"
#include "utility/gaussian_filtering/rotation.hpp"
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"


namespace module::localisation::srif {

    using utility::gaussian_filtering::Event;
    using utility::gaussian_filtering::Pose;
    using utility::gaussian_filtering::quat2rot;
    using utility::gaussian_filtering::quatXi;
    using utility::gaussian_filtering::rot2rpy;
    using utility::gaussian_filtering::rpy2rot;
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemEstimator;

    /*
     * State contains the 6-DOF torso pose in the field frame {f}, its body-fixed
     * velocity, the gyroscope bias, and a 2-DOF camera-mount attitude bias:
     *
     *     [ rBFf    ] Torso position in field frame (3)
     *     [ q       ] Torso orientation quaternion (4), Rfb = quat2rot(q)
     * x = [ vBb     ] Body-fixed translational velocity (3)
     *     [ omegaBb ] Body-fixed angular velocity (3)
     *     [ bGyro   ] Gyroscope bias, body frame (3)
     *     [ deltaC  ] Camera mount attitude bias (roll, pitch) about the camera axes (2)
     *
     * Field frame {f}: origin at centre of field on the ground plane, z up,
     * consistent with the NUbots Hfw convention.
     *
     * Pose is held in {f} and velocity in {b}, the Fossen (eta, nu) convention:
     * the velocity states are a random walk, and every velocity measurement
     * arrives body-fixed, so h(x) is the identity rather than Rfb^T.
     *
     * Nothing is a known input. The gyroscope and the walk-engine odometry are
     * measurements of omegaBb and vBb (MeasurementGyroscope,
     * MeasurementBodyVelocity), and the gyroscope bias is estimated rather than
     * calibrated.
     *
     * The process model is the kinematics plus a random walk on the rates:
     *
     *   d(rBFf)/dt = Rfb*vBb,   dq/dt = 0.5*Xi(q)*omegaBb,
     *   d(vBb)/dt  = dw_v,      d(omegaBb)/dt = dw_omega,
     *   d(bGyro)/dt = dw_b,     d(deltaC)/dt  = dw_c
     *
     * The camera bias models a constant error in the kinematic torso-to-camera
     * chain. It is a random-walk state applied on the camera side of the
     * extrinsic transform by vision measurements: Tfc = Tfb(x) * Tbc * R(deltaC)
     */
    class SystemLocalisation : public SystemEstimator {
    public:
        /**
         * @brief Process noise and input handling parameters.
         */
        struct Parameters {
            // The pose PSDs are a floor against collapse; growth comes from integrating
            // the velocity states.
            double sigmaPosXY = 0.02;  ///< Position process noise PSD, horizontal [m/sqrt(s)]
            double sigmaPosZ  = 0.01;  ///< Position process noise PSD, vertical [m/sqrt(s)]
            double sigmaAtt   = 0.01;  ///< Roll/pitch process noise PSD [rad/sqrt(s)]
            double sigmaYaw   = 0.01;  ///< Yaw process noise PSD [rad/sqrt(s)]

            // How fast the body-fixed rates may change between measurements; the
            // dominant process noise.
            double sigmaVel      = 0.35;  ///< Body linear velocity process noise PSD [m/s/sqrt(s)]
            double sigmaOmega    = 1.50;  ///< Body angular velocity process noise PSD [rad/s/sqrt(s)]
            double sigmaGyroBias = 2e-4;  ///< Gyroscope bias random walk PSD [rad/s/sqrt(s)]: thermal drift only
            double sigmaCamBias  = 3e-4;  ///< Camera mount bias process noise PSD [rad/sqrt(s)]

            // PSDs the belief decays towards while the robot is not upright, where the
            // walk-engine odometry no longer describes real motion.
            double sigmaVelDisturbed   = 1.00;  ///< Body linear velocity PSD while not upright [m/s/sqrt(s)]
            double sigmaOmegaDisturbed = 3.00;  ///< Body angular velocity PSD while not upright [rad/s/sqrt(s)]
            double sigmaPosXYDisturbed = 0.20;  ///< Horizontal position PSD while not upright [m/sqrt(s)]
            double sigmaPosZDisturbed  = 0.20;  ///< Vertical position PSD while not upright [m/sqrt(s)]
            double sigmaAttDisturbed   = 0.20;  ///< Roll/pitch PSD while not upright [rad/sqrt(s)]
            double sigmaYawDisturbed   = 0.20;  ///< Yaw PSD while not upright [rad/sqrt(s)]
            /// How long into a non-upright episode the PSDs above apply [s]. Bounds the
            /// PSDs only; odometry stays suppressed for the whole episode, see setPosture().
            double disturbedWindow = 2.0;
        };

        // State layout (nx = 18):
        //   0..2    rBFf         torso position in {f} [m]
        //   3..6    q            attitude quaternion (w, x, y, z), Rfb = quat2rot(q)
        //   7..9    vBb          body-fixed linear velocity [m/s]
        //   10..12  omegaBb      body-fixed angular velocity [rad/s]
        //   13..15  bGyro        gyroscope bias in {b} [rad/s]
        //   16..17  camera mount bias (roll, pitch) [rad]
        //
        // Attitude is a quaternion so the state stays valid through a topple, where an
        // Euler parameterisation is singular. quat2rot normalises, so |q| is invisible
        // to every geometric model and MeasurementQuaternionNorm supplies the only
        // information along it. No index means "heading": see attitudeTangent() for
        // expressing a 3-DOF attitude quantity in these four components.
        static constexpr Eigen::Index nx = 18;  ///< State dimension

        static constexpr Eigen::Index iPos      = 0;   ///< First position index
        static constexpr Eigen::Index iQuat     = 3;   ///< First quaternion index
        static constexpr Eigen::Index iVel      = 7;   ///< First body linear velocity index
        static constexpr Eigen::Index iOmega    = 10;  ///< First body angular velocity index
        static constexpr Eigen::Index iGyroBias = 13;  ///< First gyroscope bias index
        static constexpr Eigen::Index iBias     = 16;  ///< First camera-bias index

        explicit SystemLocalisation(const GaussianInfo<double>& density);
        virtual SystemLocalisation* clone() const;

        virtual void predict(double time) override;
        virtual Eigen::VectorXd dynamics(double t,
                                         const Eigen::VectorXd& x,
                                         const Eigen::VectorXd& u,
                                         Eigen::MatrixXd& J) const override;
        virtual Eigen::VectorXd input(double t, const Eigen::VectorXd& x) const override;
        virtual GaussianInfo<double> processNoiseDensity(double dt) const override;
        virtual std::vector<Eigen::Index> processNoiseIndex() const override;

        /**
         * @brief Torso pose in field frame from a state vector.
         * @param x State vector (nx)
         * @return Tfb with rotationMatrix Rfb and translationVector rBFf
         */
        template <typename Scalar>
        static Pose<Scalar> fieldPose(const Eigen::VectorX<Scalar>& x) {
            Pose<Scalar> Tfb;
            Tfb.rotationMatrix    = quat2rot(Eigen::Vector4<Scalar>(x.template segment<4>(iQuat)));
            Tfb.translationVector = x.template segment<3>(iPos);
            return Tfb;
        }

        /**
         * @brief Camera-mount attitude bias correction from a state vector.
         * @param x State vector (nx)
         * @return Rotation applied on the camera side of the extrinsic: R(deltaC)
         */
        template <typename Scalar>
        static Eigen::Matrix3<Scalar> cameraBiasRotation(const Eigen::VectorX<Scalar>& x) {
            Eigen::Vector3<Scalar> rpy;
            rpy << x(iBias), x(iBias + 1), Scalar(0);
            return rpy2rot(rpy);
        }

        /**
         * @brief Roll, pitch and yaw of the estimated attitude, for reporting.
         *
         * Always read heading through this: no state element is the yaw.
         *
         * @param x State vector (nx)
         * @return [roll, pitch, yaw] in radians
         */
        static Eigen::Vector3d attitudeRpy(const Eigen::VectorXd& x) {
            return rot2rpy(quat2rot(Eigen::Vector4d(x.segment<4>(iQuat))));
        }

        /// @brief Heading (yaw) of the estimated attitude [rad].
        static double heading(const Eigen::VectorXd& x) {
            return attitudeRpy(x)(2);
        }

        /**
         * @brief Jacobian of a body-frame rotation vector w.r.t. the quaternion states.
         *
         * A small body rotation dtheta perturbs the quaternion by dq = 0.5*Xi(q)*dtheta,
         * so this 4x3 matrix maps 3-DOF attitude quantities (process noise, yaw
         * variance, an inflation) into the four components; its pseudo-inverse maps back.
         *
         * @param x State vector (nx)
         * @return 4x3 matrix dq/dtheta at the state's attitude
         */
        static Eigen::Matrix<double, 4, 3> attitudeTangent(const Eigen::VectorXd& x) {
            Eigen::Vector4d q = x.segment<4>(iQuat);
            q.normalize();
            return 0.5 * quatXi(q);
        }

        /**
         * @brief Jacobian mapping a field-frame rotation vector to the quaternion states.
         *
         * dq = 0.5*Xi(q)*Rfb^T*dtheta_f, for uncertainties stated about a field axis,
         * chiefly yaw about field z.
         *
         * @param x State vector (nx)
         * @return 4x3 matrix dq/dtheta_f at the state's attitude
         */
        static Eigen::Matrix<double, 4, 3> attitudeTangentField(const Eigen::VectorXd& x) {
            const Eigen::Matrix3d Rfb = quat2rot(Eigen::Vector4d(x.segment<4>(iQuat)));
            return attitudeTangent(x) * Rfb.transpose();
        }

        /**
         * @brief Attitude covariance as a 3x3 in the field-frame tangent [rad^2].
         *
         * Maps the quaternion block of P back to three degrees of freedom on the field
         * axes. Element (2, 2) is the yaw variance.
         *
         * @param x State mean (nx)
         * @param P State covariance (nx by nx)
         * @return Field-tangent attitude covariance (roll, pitch, yaw)
         */
        static Eigen::Matrix3d attitudeCovariance(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
            const Eigen::Matrix<double, 3, 4> G = attitudeJacobian(x);
            return G * P.block<4, 4>(iQuat, iQuat) * G.transpose();
        }

        /**
         * @brief Left inverse of attitudeTangentField: field rotation vector per unit dq.
         *
         * Row 2 maps a quaternion perturbation to a heading change, so any position-yaw
         * cross-covariance goes through it.
         *
         * @param x State vector (nx)
         * @return 3x4 matrix dtheta_f/dq at the state's attitude
         */
        static Eigen::Matrix<double, 3, 4> attitudeJacobian(const Eigen::VectorXd& x) {
            // Left inverse is dtheta = 2*Xi^T*dq. That is 2*Xi^T, NOT
            // 2*attitudeTangent^T, which already carries the 0.5.
            Eigen::Vector4d q = x.segment<4>(iQuat);
            q.normalize();
            return quat2rot(q) * (2.0 * quatXi(q).transpose());
        }

        /// @brief Variance of the field-frame yaw implied by the attitude covariance.
        static double yawVariance(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
            return attitudeCovariance(x, P)(2, 2);
        }

        /// @brief Per-axis attitude std devs in the field tangent (roll, pitch, yaw) [rad].
        static Eigen::Vector3d attitudeStd(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
            return attitudeCovariance(x, P).diagonal().cwiseMax(0.0).cwiseSqrt();
        }

        /**
         * @brief Body-fixed linear velocity across a consecutive pair of odometry samples.
         *
         * DeltaT = Twt(a)^{-1} * Twt(b) with Twt = Htw^{-1}. That relative pose is already
         * body-frame, so its translation over dt is vBb directly.
         *
         * @param a Earlier sample
         * @param b Later sample
         * @param maxGap Maximum sample spacing to difference across [s]
         * @return vBb [m/s], or a non-finite vector when the pair cannot be differenced.
         */
        static Eigen::Vector3d bodyVelocityFromOdometry(const SensorsSample& a,
                                                        const SensorsSample& b,
                                                        double maxGap = 0.1);

        /// @brief Body-fixed linear velocity of a state [m/s].
        static Eigen::Vector3d bodyVelocity(const Eigen::VectorXd& x) {
            return x.segment<3>(iVel);
        }

        /// @brief Body-fixed angular velocity of a state [rad/s].
        static Eigen::Vector3d bodyRate(const Eigen::VectorXd& x) {
            return x.segment<3>(iOmega);
        }

        /// @brief Estimated gyroscope bias of a state [rad/s].
        static Eigen::Vector3d gyroBias(const Eigen::VectorXd& x) {
            return x.segment<3>(iGyroBias);
        }

        /**
         * @brief Reset the state density and system clock (initialisation / relocalisation).
         *
         * Collapses any active hypothesis mixture. Seed the bank afterwards
         * (initialiseHypotheses) if the new belief is still symmetry-ambiguous.
         *
         * @param density New state density
         * @param time New system time [s]
         */
        void resetTo(const GaussianInfo<double>& density, double time);

        /**
         * @brief Declare the robot's posture for this step.
         *
         * A fall has two consequences with different lifetimes, so they are separate
         * switches. Odometry velocity is meaningless for the whole time the robot is not
         * upright, and is suppressed by the caller, which substitutes a zero-velocity
         * update for MeasurementBodyVelocity. This method carries the other half: the
         * process noise switches to the `*Disturbed` PSDs for params.disturbedWindow
         * seconds from the start of the episode, then stands down.
         *
         * A mode, not an event: the caller sets it every frame from the posture.
         *
         * @param upright True when the robot is upright
         * @param disturbedFor Seconds since the current non-upright episode began (ignored
         *                     when upright; 0 means "just started", i.e. fully disturbed)
         */
        void setPosture(bool upright, double disturbedFor = 0.0) {
            diffusing_ = !upright && !(disturbedFor >= params.disturbedWindow);
        }

        /**
         * @brief Add variance to the belief without moving its mean.
         *
         * Used on recovery from a fall, where the pre-fall mean is still the best
         * estimate but its confidence is not. Applies to every live hypothesis as well
         * as to the representative density.
         *
         * @param extraVar Variance to add per state element (length nx, non-negative)
         */
        void inflateCovariance(const Eigen::VectorXd& extraVar);

        /**
         * @brief Add a full covariance block to the belief without moving its mean.
         *
         * Needed for attitude: yaw uncertainty about the field z axis lands on the
         * quaternion states as a rank-one block (attitudeTangentField), not one element.
         *
         * @param extraCov Positive-semidefinite matrix (nx by nx) added to the covariance
         */
        void inflateCovariance(const Eigen::MatrixXd& extraCov);

        /**
         * @brief Project the attitude mean back onto the unit sphere (and w >= 0).
         *
         * Called after every predict and every measurement update; the soft prior from
         * MeasurementQuaternionNorm is not enough on its own to hold |q| = 1.
         */
        void normaliseQuaternion();

        /**
         * @brief 180 deg field rotation as a linear map on the quaternion components.
         *
         * qz(pi) (x) q for qz(pi) = (0, 0, 0, 1) sends (w, x, y, z) to (-z, -y, x, w).
         */
        static Eigen::Matrix4d mirrorQuatMap() {
            Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
            M(0, 3)           = -1.0;
            M(1, 2)           = -1.0;
            M(2, 1)           = 1.0;
            M(3, 0)           = 1.0;
            return M;
        }

        /**
         * @brief Per-component process-noise std devs for the quaternion block.
         *
         * The PSDs are specified in the 3-DOF body tangent, where a tangent std of s
         * becomes a component std of s/2. The fourth (radial) component takes the same
         * magnitude, leaving MeasurementQuaternionNorm room to work.
         *
         * @param sigmaAtt Roll/pitch process noise PSD [rad/sqrt(s)]
         * @param sigmaYaw Yaw process noise PSD [rad/sqrt(s)]
         */
        static Eigen::Vector4d quaternionSigma(double sigmaAtt, double sigmaYaw) {
            const double s = 0.5 * std::max(sigmaAtt, sigmaYaw);
            return Eigen::Vector4d::Constant(s);
        }

        /**
         * @brief Advance the belief to @p time with no measurement.
         *
         * Prediction otherwise only happens inside Event::process, so a frame with no
         * usable measurement would advance neither the state nor the clock. Predicts
         * every hypothesis when the bank is active.
         *
         * @param time Time to advance the belief to [s]
         */
        void predictAll(double time);

        Parameters params;

        // ---------------------------------------------------------------------
        // Hypothesis bank (multi-hypothesis field-symmetry handling)
        //
        // The RoboCup field has a 180 deg rotational symmetry about its centre, so a
        // pose and its mirror produce identical landmark observations and a single
        // Gaussian cannot represent the belief while the symmetry is unbroken. The
        // belief is instead a weighted Gaussian mixture (cf. B-Human, Rofer et al.):
        // each component is an independent pose density run through the same
        // predict/update machinery, weighted by the Laplace log-evidence each
        // measurement reports (Measurement::logEvidence()).
        //
        // With no hypotheses active (components_ empty) the estimator is exactly a
        // single-Gaussian filter over `density`. Once the bank is active, `density`
        // tracks the maximum-weight component so single-Gaussian accessors keep working.
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
         * Seeds two equally weighted hypotheses so a symmetry ambiguity present at
         * initialisation can be resolved by later asymmetric evidence; the wrong mirror
         * is down-weighted and pruned automatically.
         */
        void initialiseHypotheses();

        /**
         * @brief Add the 180 deg mirror of the maximum-weight component as a new,
         *        equally weighted hypothesis (used for symmetry-flip recovery).
         */
        void spawnMirror();

        /**
         * @brief Fold one frame of out-of-field side evidence into the mixture weights.
         *
         * On-field landmarks fit both symmetric hypotheses equally well, so only the
         * asymmetric background scenery can separate them. @p logRatio (SideDisambiguator's
         * clamped own-minus-mirror score, FrameResult::sideDelta) is added to the
         * representative component's log-weight: a sustained positive ratio collapses the
         * mirror, a negative one hands leadership to it.
         *
         * No-op unless at least two hypotheses are live.
         *
         * @param logRatio Log-likelihood ratio own-vs-mirror for this frame [nats]
         */
        void addSideLogEvidence(double logRatio);

        /**
         * @brief Process an event across every active hypothesis.
         *
         * For each component the shared system clock is rewound and the event applied
         * through the ordinary single-Gaussian path, with measurement events also
         * accumulating their Laplace log-evidence into the component weight. The mixture
         * is then normalised, merged, pruned, respawned if it has collapsed to a single
         * uncertain component, and `density` set to the maximum-weight component.
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
         *   (x, y) -> (-x, -y),  q -> mirrorQuatMap()*q,  z/cam-bias unchanged.
         */
        static Eigen::VectorXd mirrorState(const Eigen::VectorXd& x);

        /**
         * @brief The 180 deg field-symmetry mirror of a pose density.
         */
        static GaussianInfo<double> mirrorDensity(const GaussianInfo<double>& g);

    protected:
        // No input buffer: the process model is autonomous, and every would-be input is
        // a measurement of the corresponding state.
        bool diffusing_ = false;  ///< Elevated fall PSDs in force (bounded window, see setPosture)

        std::vector<GaussianInfo<double>> components_;  ///< Mixture components (empty => single-hypothesis)
        std::vector<double> logWeights_;                ///< Unnormalised log weights per component
        Eigen::VectorXd lastRepMean_;                   ///< Outgoing representative mean (setRepresentative hysteresis)

        void normaliseWeights();      ///< Renormalise logWeights_ (subtract log-sum-exp)
        void mergeComponents();       ///< Merge components within the merge gate (keep-best)
        void pruneComponents();       ///< Drop low-weight components; cap to maxComponents
        void respawnIfUnconfident();  ///< Respawn a mirror if collapsed to one uncertain component
        void setRepresentative();     ///< Set `density` to the maximum-weight component (tie-broken by hysteresis)
    };

}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_SYSTEMLOCALISATION_HPP
