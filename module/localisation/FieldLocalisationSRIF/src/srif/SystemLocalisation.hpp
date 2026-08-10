/**
 * @file SystemLocalisation.h
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

    /**
     * @brief A body-fixed twist sample derived from odometry.
     */
    struct BodyTwistSample {
        double t;                 ///< Sample time [s] (relative to log start)
        Eigen::Vector3d vBb;      ///< Body-fixed translational velocity [m/s]
        Eigen::Vector3d omegaBb;  ///< Body-fixed angular velocity [rad/s]
    };

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
     * Pose in {f}, velocity in {b}: the Fossen vehicle-dynamics convention
     * (eta, nu) rather than the strapdown-INS one, which would carry velocity in
     * {f} alongside position. Two reasons. The velocity states are a random walk,
     * and "the robot keeps walking forward at this speed" is a far better model of
     * a turning robot than "its field-frame velocity vector is constant" -- the
     * latter needs a PSD sized for the turn even when walking straight. And every
     * velocity measurement arrives body-fixed (the gyroscope directly, the walk
     * odometry as a body-frame finite difference), so h(x) is the identity rather
     * than Rfb^T. The frames are mixed regardless: omegaBb has to be body-fixed
     * because that is what the gyroscope measures and what q-dot takes.
     *
     * A pleasant consequence: the 180 deg field mirror leaves both velocity blocks
     * alone. v_b' = (Rz(pi) Rfb)^T Rz(pi) v_f = v_b, so mirrorState() stays a
     * position-and-quaternion operation. Field-frame velocity would have needed
     * its horizontal components negated alongside the position.
     *
     * Nothing is a known input. The gyroscope and the walk-engine odometry are
     * *measurements* of omegaBb and vBb (MeasurementGyroscope,
     * MeasurementBodyVelocity), so their noise is modelled where it belongs and
     * the gyroscope bias is estimated rather than calibrated by heuristic. That
     * bias is unobservable to the upstream Mahony filter, whose bias integrator is
     * driven by the gravity error -- a cross product of two near-vertical vectors,
     * which has no component about the vertical -- so before this nothing in the
     * system estimated the yaw-rate bias, the one that becomes heading drift.
     * Measured on the data2 recording it is ~0.78 deg/s, and estimating it is
     * worth 2.5 deg of heading RMSE.
     *
     * The process model is the kinematics plus a random walk on the rates:
     *
     *   d(rBFf)/dt = Rfb*vBb,   dq/dt = 0.5*Xi(q)*omegaBb,
     *   d(vBb)/dt  = dw_v,      d(omegaBb)/dt = dw_omega,
     *   d(bGyro)/dt = dw_b,     d(deltaC)/dt  = dw_c
     *
     * The camera bias models a constant error in the kinematic torso-to-camera
     * chain (evident in the recorded data as ground-projection errors growing
     * with range^2, ~1 m at 4-5 m range, consistent with a 1.5-2 deg pitch
     * bias). It is a random-walk state with very small process noise, applied
     * on the camera side of the extrinsic transform by vision measurements:
     *   Tfc = Tfb(x) * Tbc * R(deltaC)
     */
    class SystemLocalisation : public SystemEstimator {
    public:
        /**
         * @brief Process noise and input handling parameters.
         */
        struct Parameters {
            // The pose states are driven by the velocity states, so their own PSDs are a
            // small floor against collapse in weakly-observable directions rather than
            // the main source of growth: uncertainty now reaches position by integrating
            // the velocity uncertainty, which is where the real ignorance lives.
            double sigmaPosXY = 0.02;  ///< Position process noise PSD, horizontal [m/sqrt(s)]
            double sigmaPosZ  = 0.01;  ///< Position process noise PSD, vertical [m/sqrt(s)]
            double sigmaAtt   = 0.01;  ///< Roll/pitch process noise PSD [rad/sqrt(s)]
            double sigmaYaw   = 0.01;  ///< Yaw process noise PSD [rad/sqrt(s)]

            // How fast the body-fixed rates are allowed to change between measurements.
            // These are the dominant process noise now. A walking robot changes its
            // forward speed over a step (~0.3 s), so a PSD near the walk speed itself is
            // about right; angular velocity is measured at IMU rate so its random walk
            // only has to cover one sample interval.
            double sigmaVel      = 0.35;  ///< Body linear velocity process noise PSD [m/s/sqrt(s)]
            double sigmaOmega    = 1.50;  ///< Body angular velocity process noise PSD [rad/s/sqrt(s)]
            double sigmaGyroBias = 2e-4;  ///< Gyroscope bias random walk PSD [rad/s/sqrt(s)]: slow thermal
                                          ///< drift only, so vision can average it out over many frames
            double sigmaCamBias = 3e-4;   ///< Camera mount bias process noise PSD [rad/sqrt(s)] (kept small
                                          ///< so bias cannot wander while stationary)

            // While the robot is not upright the rates are not merely noisier, they are
            // unmeasured: the walk-engine odometry describes a gait that is not
            // happening, so MeasurementBodyVelocity is replaced by a zero-velocity
            // update. These PSDs are what the belief decays towards meanwhile, so a fall
            // widens honestly enough to reopen the landmark association gates on recovery.
            double sigmaVelDisturbed   = 1.00;  ///< Body linear velocity PSD while not upright [m/s/sqrt(s)]
            double sigmaOmegaDisturbed = 3.00;  ///< Body angular velocity PSD while not upright [rad/s/sqrt(s)]
            double sigmaPosXYDisturbed = 0.20;  ///< Horizontal position PSD while not upright [m/sqrt(s)]
            double sigmaPosZDisturbed  = 0.20;  ///< Vertical position PSD while not upright [m/sqrt(s)]
            double sigmaAttDisturbed   = 0.20;  ///< Roll/pitch PSD while not upright [rad/sqrt(s)]
            double sigmaYawDisturbed   = 0.20;  ///< Yaw PSD while not upright [rad/sqrt(s)]
            /// How long into a non-upright episode the PSDs above apply [s]. Past this the
            /// robot is lying still and diffusing further would be inventing motion. Note
            /// this bounds the PSDs ONLY -- the odometry velocity measurement stays
            /// suppressed for the whole episode, see setPosture().
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
        // Attitude was roll-pitch-yaw until the fall work. The Euler-rate transform
        // TK() is singular at pitch = +-90 deg, which is not an edge case for a
        // falling robot -- it is on the trajectory of every topple. Passing through
        // it landed the state on the gimbal alias (roll+180, 180-pitch, yaw+180):
        // the same rotation, so fieldPose() and the landmark models carried on
        // working, but every consumer reading x(5) as heading was then 180 deg out.
        // A quaternion has no such point, so the state can sit at pitch = 90 deg for
        // as long as the robot is face-down and measurement updates can keep running
        // there.
        //
        // The cost is a fourth parameter for three degrees of freedom. quat2rot
        // normalises, so |q| is invisible to every geometric model; the information
        // along it comes from MeasurementQuaternionNorm alone, which is what keeps
        // the MAP Hessian non-singular. See attitudeTangent() for the mapping used
        // wherever a 3-DOF attitude quantity (process noise, yaw variance, a yaw
        // inflation) has to be expressed in these four components.
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
         * Always read heading through this rather than off a state element. The
         * quaternion has no distinguished yaw component, and the whole point of the
         * change is that no single index means "heading" any more.
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
         * so this 4x3 matrix maps 3-DOF attitude quantities into the four components
         * and its pseudo-inverse maps them back. Everything that used to index the
         * single yaw element -- process noise, the association gate's yaw variance,
         * the recovery inflation -- goes through it.
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
         * dq = 0.5*Xi(q)*Rfb^T*dtheta_f. Used wherever an uncertainty is naturally
         * stated about a field axis -- above all yaw, about field z.
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
         * The replacement for reading P(3..5, 3..5) directly. Xi has orthonormal
         * columns for a unit q, so the pseudo-inverse of dq/dtheta is 2*Xi^T; that
         * maps the quaternion block of P back to three degrees of freedom, and Rfb
         * puts them on the field axes. Element (2, 2) is the yaw variance.
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
         * Row 2 is what maps a quaternion perturbation to a heading change, so it is
         * also what any cross-covariance between position and yaw has to go through
         * (the Field message's reported (x, y, yaw) block, for one).
         *
         * @param x State vector (nx)
         * @return 3x4 matrix dtheta_f/dq at the state's attitude
         */
        static Eigen::Matrix<double, 3, 4> attitudeJacobian(const Eigen::VectorXd& x) {
            // dq = 0.5*Xi*dtheta and Xi has orthonormal columns, so the left inverse
            // is dtheta = 2*Xi^T*dq. Note that is 2*Xi^T, NOT 2*attitudeTangent^T --
            // attitudeTangent already carries the 0.5, and folding it in twice
            // under-reports every attitude std dev by a factor of two.
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
         * @brief Body-fixed velocity samples finite-differenced from the odometry stream.
         *
         * For consecutive odometry samples, the relative pose
         * DeltaT = Twt(t1)^{-1} * Twt(t2) with Twt = Htw^{-1} yields
         * vBb = Delta r / dt, stamped at the interval midpoint; omegaBb is
         * log(Delta R) / dt. Samples spanning gaps larger than maxGap are skipped.
         *
         * These are now *measurements* (MeasurementBodyVelocity), not a known input,
         * so no gyroscope enters here and no bias is subtracted: the raw gyroscope is
         * its own measurement (MeasurementGyroscope) and its bias is a state. The
         * omegaBb field is retained for the odometry-derived angular rate, which is
         * only of interest as a fallback when no gyroscope is available.
         *
         * @param sensors Time-ordered odometry samples (absolute time [s])
         * @param t0 Time origin subtracted from sample times [s]
         * @param maxGap Maximum sample spacing to difference across [s]
         */
        static std::vector<BodyTwistSample> twistFromOdometry(const std::vector<SensorsSample>& sensors,
                                                              double t0,
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
         * A reset asserts a single known belief, so it also collapses any active
         * hypothesis mixture. Seed the bank afterwards (initialiseHypotheses) if the
         * new belief is still symmetry-ambiguous.
         *
         * @param density New state density
         * @param time New system time [s]
         */
        void resetTo(const GaussianInfo<double>& density, double time);

        /**
         * @brief Declare the robot's posture for this step.
         *
         * Two different things follow from a fall, and they are deliberately NOT the same
         * switch, because they expire differently:
         *
         *  - The twist's linear velocity is discarded. It is derived by differencing
         *    walk-engine odometry, which while the robot is on the ground describes a gait
         *    that is not happening -- and during a getup describes a scripted flail that is
         *    happening but is not locomotion. That is a statement about whether the signal
         *    means anything, and it does not become true again after some number of
         *    seconds: it holds for the WHOLE time the robot is not upright. The
         *    gyroscope-derived angular velocity is kept throughout, because it measures the
         *    topple for real. That half is enforced by the caller, which substitutes a
         *    zero-velocity update for MeasurementBodyVelocity while not upright; this
         *    method carries only the process-noise half.
         *
         *  - The process noise switches to the `*Disturbed` PSDs, so the belief decays
         *    honestly instead of coasting at walking-grade confidence. That one IS bounded:
         *    a fall is a bounded event, and modelling a robot lying still as a 0.40 m/sqrt(s)
         *    random walk would make the belief's width report how long it had been down
         *    rather than how far it could have gone. It applies for params.disturbedWindow
         *    seconds from the start of the episode and then stands down.
         *
         * Tying both to the same window is what let a long fall integrate the getup's
         * odometry at nominal confidence, marching the estimate off the field with a
         * covariance too tight for the association gate to ever recover it.
         *
         * This is a mode, not an event: the caller sets it every frame from the posture.
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
         * Used on recovery from a fall. The pre-fall mean is still the best estimate
         * available -- a fall and getup translate the torso well under a metre, far
         * less than a global relocalisation would risk getting wrong -- but the
         * confidence attached to it is not survivable, particularly in yaw. Applies
         * to every live hypothesis as well as to the representative density.
         *
         * @param extraVar Variance to add per state element (length nx, non-negative)
         */
        void inflateCovariance(const Eigen::VectorXd& extraVar);

        /**
         * @brief Add a full covariance block to the belief without moving its mean.
         *
         * The diagonal overload cannot express an attitude inflation any more: yaw
         * uncertainty about the field z axis lands on the quaternion states as a
         * rank-one block (attitudeTangentField), not on one element.
         *
         * @param extraCov Positive-semidefinite matrix (nx by nx) added to the covariance
         */
        void inflateCovariance(const Eigen::MatrixXd& extraCov);

        /**
         * @brief Project the attitude mean back onto the unit sphere (and w >= 0).
         *
         * Called after every predict and every measurement update.
         * MeasurementQuaternionNorm keeps the belief near the sphere but is a soft
         * prior, so this is what actually holds |q| = 1.
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
         * The PSDs are specified in the 3-DOF body tangent (roll/pitch and yaw), which
         * is where they are meaningful. A small body rotation dtheta moves the
         * quaternion by 0.5*Xi(q)*dtheta and Xi has orthonormal columns, so a tangent
         * std of s becomes a component std of s/2. The fourth (radial) component is
         * given the same magnitude as the attitude channels: it is invisible to every
         * geometric model, so its only job is to leave MeasurementQuaternionNorm room
         * to work rather than to fight it.
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
         * Prediction otherwise only ever happens inside Event::process, so a frame
         * that yields no usable measurement used to advance neither the state nor the
         * clock. That is exactly what a fall produces (the camera is in the carpet and
         * YOLO returns nothing), and it left the filter holding its pre-fall mean at
         * its pre-fall covariance across the whole event. Predicts every hypothesis
         * when the bank is active.
         *
         * @param time Time to advance the belief to [s]
         */
        void predictAll(double time);

        Parameters params;

        // ---------------------------------------------------------------------
        // Hypothesis bank (multi-hypothesis field-symmetry handling)
        //
        // The RoboCup field has a 180 deg rotational symmetry about its centre:
        // the pose (rBFf, q) and its mirror produce identical landmark
        // observations, so a single Gaussian cannot represent the true belief
        // when the symmetry is unbroken. Following the B-Human multi-hypothesis
        // approach (Rofer et al.), the belief is a weighted Gaussian mixture; each
        // component is an independent pose density carried through the same
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
         * @brief Fold one frame of out-of-field side evidence into the mixture weights.
         *
         * On-field landmarks cannot separate the two symmetric hypotheses (they fit
         * their respective mirror-partner landmarks equally well), so the mixture
         * would sit at 50/50 forever on landmark evidence alone. The asymmetric
         * background scenery is what breaks it: @p logRatio is the per-frame
         * log-likelihood ratio favouring the representative pose over its 180 deg
         * mirror (SideDisambiguator's clamped own-minus-mirror score, FrameResult::
         * sideDelta). It is added to the representative component's log-weight, so a
         * sustained positive ratio collapses the mirror and a negative one hands
         * leadership to it.
         *
         * No-op unless at least two hypotheses are live (nothing to disambiguate).
         *
         * @param logRatio Log-likelihood ratio own-vs-mirror for this frame [nats]
         */
        void addSideLogEvidence(double logRatio);

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
         *   (x, y) -> (-x, -y),  q -> mirrorQuatMap()*q,  z/cam-bias unchanged.
         */
        static Eigen::VectorXd mirrorState(const Eigen::VectorXd& x);

        /**
         * @brief The 180 deg field-symmetry mirror of a pose density.
         */
        static GaussianInfo<double> mirrorDensity(const GaussianInfo<double>& g);

    protected:
        // No input buffer: the process model is autonomous (kinematics plus a random
        // walk on the rates), and everything that used to be a known input is now a
        // measurement of the corresponding state.
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
