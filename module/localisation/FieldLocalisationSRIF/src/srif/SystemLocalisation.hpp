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
#include "utility/gaussian_filtering/system/SystemEstimator.hpp"
#include "utility/math/euler.hpp"
#include "utility/math/quaternion.hpp"


namespace module::localisation::srif {

    using utility::gaussian_filtering::Event;
    using utility::gaussian_filtering::Pose;
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::gaussian_filtering::measurement::Measurement;
    using utility::gaussian_filtering::system::SystemEstimator;
    using utility::math::euler::mat_to_rpy_intrinsic;
    using utility::math::euler::rpy_intrinsic_to_mat;
    using utility::math::quaternion::wxyz_kinematics;
    using utility::math::quaternion::wxyz_to_mat;

    /*
     * State contains the 6-DOF torso pose in the field frame {f}, its body-fixed
     * velocity, the gyroscope bias, and a 2-DOF camera-mount attitude bias:
     *
     *     [ rBFf    ] Torso position in field frame (3)
     *     [ q       ] Torso orientation quaternion (4), Rfb = wxyz_to_mat(q)
     * x = [ vBb     ] Body-fixed translational velocity (3)
     *     [ omegaBb ] Body-fixed angular velocity (3)
     *     [ bGyro   ] Gyroscope bias, body frame (3)
     *     [ deltaC  ] Camera mount attitude bias (roll, pitch) about the camera axes (2)
     *
     * Field frame {f}: origin at centre of field on the ground plane, z up,
     * consistent with the NUbots Hfw convention.
     *
     * Pose is held in {f} and velocity in {b} (Fossen's eta, nu), so body-fixed velocity
     * measurements have h(x) = identity. There is no known input: the gyroscope and odometry
     * are measurements of omegaBb and vBb.
     *
     * Process model:
     *
     *   d(rBFf)/dt = Rfb*vBb,   dq/dt = 0.5*Xi(q)*omegaBb,
     *   d(vBb)/dt  = dw_v,      d(omegaBb)/dt = dw_omega,
     *   d(bGyro)/dt = dw_b,     d(deltaC)/dt  = dw_c
     *
     * The camera bias models a constant error in the torso-to-camera chain, applied as
     * Tfc = Tfb(x) * Tbc * R(deltaC).
     */
    class SystemLocalisation : public SystemEstimator {
    public:
        /**
         * @brief Process noise parameters.
         */
        struct Parameters {
            // Pose PSDs are a floor; growth comes from integrating the velocity states
            double sigma_pos_xy = 0.02;  ///< Position process noise PSD, horizontal [m/sqrt(s)]
            double sigma_pos_z  = 0.01;  ///< Position process noise PSD, vertical [m/sqrt(s)]
            double sigma_att    = 0.01;  ///< Roll/pitch process noise PSD [rad/sqrt(s)]
            double sigma_yaw    = 0.01;  ///< Yaw process noise PSD [rad/sqrt(s)]

            // How fast the body-fixed rates may change; the dominant process noise
            double sigma_vel       = 0.35;  ///< Body linear velocity process noise PSD [m/s/sqrt(s)]
            double sigma_omega     = 1.50;  ///< Body angular velocity process noise PSD [rad/s/sqrt(s)]
            double sigma_gyro_bias = 2e-4;  ///< Gyroscope bias random walk PSD [rad/s/sqrt(s)]: thermal drift only
            double sigma_cam_bias  = 3e-4;  ///< Camera mount bias process noise PSD [rad/sqrt(s)]

            // PSDs while the robot is not upright
            double sigma_vel_disturbed    = 1.00;  ///< Body linear velocity PSD while not upright [m/s/sqrt(s)]
            double sigma_omega_disturbed  = 3.00;  ///< Body angular velocity PSD while not upright [rad/s/sqrt(s)]
            double sigma_pos_xy_disturbed = 0.20;  ///< Horizontal position PSD while not upright [m/sqrt(s)]
            double sigma_pos_z_disturbed  = 0.20;  ///< Vertical position PSD while not upright [m/sqrt(s)]
            double sigma_att_disturbed    = 0.20;  ///< Roll/pitch PSD while not upright [rad/sqrt(s)]
            double sigma_yaw_disturbed    = 0.20;  ///< Yaw PSD while not upright [rad/sqrt(s)]
            /// How long into a non-upright episode the PSDs above apply [s]. Bounds the
            /// PSDs only; odometry stays suppressed for the whole episode, see set_posture().
            double disturbed_window = 2.0;
        };

        // State layout (nx = 18):
        //   0..2    rBFf         torso position in {f} [m]
        //   3..6    q            attitude quaternion (w, x, y, z), Rfb = wxyz_to_mat(q)
        //   7..9    vBb          body-fixed linear velocity [m/s]
        //   10..12  omegaBb      body-fixed angular velocity [rad/s]
        //   13..15  bGyro        gyroscope bias in {b} [rad/s]
        //   16..17  camera mount bias (roll, pitch) [rad]
        //
        // A quaternion stays valid through a topple, where Euler angles are singular.
        // wxyz_to_mat normalises, so only MeasurementQuaternionNorm informs |q|. No index is the
        // heading; use heading() and attitude_tangent().
        static constexpr Eigen::Index nx = 18;  ///< State dimension

        static constexpr Eigen::Index i_pos       = 0;   ///< First position index
        static constexpr Eigen::Index i_quat      = 3;   ///< First quaternion index
        static constexpr Eigen::Index i_vel       = 7;   ///< First body linear velocity index
        static constexpr Eigen::Index i_omega     = 10;  ///< First body angular velocity index
        static constexpr Eigen::Index i_gyro_bias = 13;  ///< First gyroscope bias index
        static constexpr Eigen::Index i_bias      = 16;  ///< First camera-bias index

        explicit SystemLocalisation(const GaussianInfo<double>& density);
        virtual SystemLocalisation* clone() const;

        virtual void predict(double time) override;
        virtual Eigen::VectorXd dynamics(double t,
                                         const Eigen::VectorXd& x,
                                         const Eigen::VectorXd& u,
                                         Eigen::MatrixXd& J) const override;
        virtual Eigen::VectorXd input(double t, const Eigen::VectorXd& x) const override;
        virtual GaussianInfo<double> process_noise_density(double dt) const override;
        virtual std::vector<Eigen::Index> process_noise_index() const override;

        /**
         * @brief Torso pose in field frame from a state vector.
         * @param x State vector (nx)
         * @return Tfb with rotation_matrix Rfb and translation_vector rBFf
         */
        template <typename Scalar>
        static Pose<Scalar> field_pose(const Eigen::VectorX<Scalar>& x) {
            Pose<Scalar> Tfb;
            Tfb.rotation_matrix    = wxyz_to_mat(Eigen::Vector4<Scalar>(x.template segment<4>(i_quat)));
            Tfb.translation_vector = x.template segment<3>(i_pos);
            return Tfb;
        }

        /**
         * @brief Camera-mount attitude bias correction from a state vector.
         * @param x State vector (nx)
         * @return Rotation applied on the camera side of the extrinsic: R(deltaC)
         */
        template <typename Scalar>
        static Eigen::Matrix3<Scalar> camera_bias_rotation(const Eigen::VectorX<Scalar>& x) {
            Eigen::Vector3<Scalar> rpy;
            rpy << x(i_bias), x(i_bias + 1), Scalar(0);
            return rpy_intrinsic_to_mat(rpy);
        }

        /**
         * @brief Roll, pitch and yaw of the estimated attitude, for reporting.
         *
         * Always read heading through this: no state element is the yaw.
         *
         * @param x State vector (nx)
         * @return [roll, pitch, yaw] in radians
         */
        static Eigen::Vector3d attitude_rpy(const Eigen::VectorXd& x) {
            return mat_to_rpy_intrinsic(wxyz_to_mat(Eigen::Vector4d(x.segment<4>(i_quat))));
        }

        /// @brief Heading (yaw) of the estimated attitude [rad].
        static double heading(const Eigen::VectorXd& x) {
            return attitude_rpy(x)(2);
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
        static Eigen::Matrix<double, 4, 3> attitude_tangent(const Eigen::VectorXd& x) {
            Eigen::Vector4d q = x.segment<4>(i_quat);
            q.normalize();
            return 0.5 * wxyz_kinematics(q);
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
        static Eigen::Matrix<double, 4, 3> attitude_tangent_field(const Eigen::VectorXd& x) {
            const Eigen::Matrix3d Rfb = wxyz_to_mat(Eigen::Vector4d(x.segment<4>(i_quat)));
            return attitude_tangent(x) * Rfb.transpose();
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
        static Eigen::Matrix3d attitude_covariance(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
            const Eigen::Matrix<double, 3, 4> G = attitude_jacobian(x);
            return G * P.block<4, 4>(i_quat, i_quat) * G.transpose();
        }

        /**
         * @brief Left inverse of attitude_tangent_field: field rotation vector per unit dq.
         *
         * Row 2 maps a quaternion perturbation to a heading change, so any position-yaw
         * cross-covariance goes through it.
         *
         * @param x State vector (nx)
         * @return 3x4 matrix dtheta_f/dq at the state's attitude
         */
        static Eigen::Matrix<double, 3, 4> attitude_jacobian(const Eigen::VectorXd& x) {
            // Left inverse is dtheta = 2*Xi^T*dq. That is 2*Xi^T, NOT
            // 2*attitude_tangent^T, which already carries the 0.5.
            Eigen::Vector4d q = x.segment<4>(i_quat);
            q.normalize();
            return wxyz_to_mat(q) * (2.0 * wxyz_kinematics(q).transpose());
        }

        /// @brief Variance of the field-frame yaw implied by the attitude covariance.
        static double yaw_variance(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
            return attitude_covariance(x, P)(2, 2);
        }

        /// @brief Per-axis attitude std devs in the field tangent (roll, pitch, yaw) [rad].
        static Eigen::Vector3d attitude_std(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
            return attitude_covariance(x, P).diagonal().cwiseMax(0.0).cwiseSqrt();
        }

        /**
         * @brief Body-fixed linear velocity across a consecutive pair of odometry samples.
         *
         * DeltaT = Twt(a)^{-1} * Twt(b) with Twt = Htw^{-1}. That relative pose is already
         * body-frame, so its translation over dt is vBb directly.
         *
         * @param a Earlier sample
         * @param b Later sample
         * @param max_gap Maximum sample spacing to difference across [s]
         * @return vBb [m/s], or a non-finite vector when the pair cannot be differenced.
         */
        static Eigen::Vector3d body_velocity_from_odometry(const SensorsSample& a,
                                                           const SensorsSample& b,
                                                           double max_gap = 0.1);

        /// @brief Body-fixed linear velocity of a state [m/s].
        static Eigen::Vector3d body_velocity(const Eigen::VectorXd& x) {
            return x.segment<3>(i_vel);
        }

        /// @brief Body-fixed angular velocity of a state [rad/s].
        static Eigen::Vector3d body_rate(const Eigen::VectorXd& x) {
            return x.segment<3>(i_omega);
        }

        /// @brief Estimated gyroscope bias of a state [rad/s].
        static Eigen::Vector3d gyro_bias(const Eigen::VectorXd& x) {
            return x.segment<3>(i_gyro_bias);
        }

        /**
         * @brief Reset the state density and system clock (initialisation / relocalisation).
         *
         * Collapses any active hypothesis mixture. Seed the bank afterwards
         * (initialise_hypotheses) if the new belief is still symmetry-ambiguous.
         *
         * @param density New state density
         * @param time New system time [s]
         */
        void reset_to(const GaussianInfo<double>& density, double time);

        /**
         * @brief Declare the robot's posture for this step.
         *
         * While not upright the process noise uses the `*_disturbed` PSDs for the first
         * params.disturbed_window seconds. Odometry suppression is the caller's job. Set
         * every frame.
         *
         * @param upright True when the robot is upright
         * @param disturbed_for Seconds since the current non-upright episode began (ignored
         *                     when upright; 0 means "just started", i.e. fully disturbed)
         */
        void set_posture(bool upright, double disturbed_for = 0.0) {
            diffusing_ = !upright && !(disturbed_for >= params.disturbed_window);
        }

        /**
         * @brief Add variance to the belief without moving its mean.
         *
         * Applies to every live hypothesis. Used on fall recovery.
         *
         * @param extra_var Variance to add per state element (length nx, non-negative)
         */
        void inflate_covariance(const Eigen::VectorXd& extra_var);

        /**
         * @brief Add a full covariance block to the belief without moving its mean.
         *
         * Needed for attitude: yaw uncertainty about the field z axis lands on the
         * quaternion states as a rank-one block (attitude_tangent_field), not one element.
         *
         * @param extra_cov Positive-semidefinite matrix (nx by nx) added to the covariance
         */
        void inflate_covariance(const Eigen::MatrixXd& extra_cov);

        /**
         * @brief Project the attitude mean back onto the unit sphere (and w >= 0).
         *
         * Called after every predict and every measurement update; the soft prior from
         * MeasurementQuaternionNorm is not enough on its own to hold |q| = 1.
         */
        void normalise_quaternion();

        /**
         * @brief 180 deg field rotation as a linear map on the quaternion components.
         *
         * qz(pi) (x) q for qz(pi) = (0, 0, 0, 1) sends (w, x, y, z) to (-z, -y, x, w).
         */
        static Eigen::Matrix4d mirror_quat_map() {
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
         * @param sigma_att Roll/pitch process noise PSD [rad/sqrt(s)]
         * @param sigma_yaw Yaw process noise PSD [rad/sqrt(s)]
         */
        static Eigen::Vector4d quaternion_sigma(double sigma_att, double sigma_yaw) {
            const double s = 0.5 * std::max(sigma_att, sigma_yaw);
            return Eigen::Vector4d::Constant(s);
        }

        /**
         * @brief Advance the belief to @p time with no measurement.
         *
         * Predicts every hypothesis when the bank is active.
         *
         * @param time Time to advance the belief to [s]
         */
        void predict_all(double time);

        Parameters params;

        // ---------------------------------------------------------------------
        // Hypothesis bank (multi-hypothesis field-symmetry handling)
        //
        // The field's 180 deg symmetry makes a pose and its mirror indistinguishable to
        // landmarks, so the belief may be a Gaussian mixture weighted by each measurement's
        // Laplace log-evidence. With components_ empty this is a single-Gaussian filter over
        // `density`; otherwise `density` is the representative component.
        // ---------------------------------------------------------------------

        /**
         * @brief Parameters governing hypothesis spawning, pruning and merging.
         */
        struct HypothesisParameters {
            double min_weight          = 0.02;  ///< Prune components below this normalised weight
            std::size_t max_components = 4;     ///< Cap on the number of live components
            double merge_position      = 0.30;  ///< Merge gate on position separation [m]
            double merge_yaw           = 0.20;  ///< Merge gate on yaw separation [rad]
            double respawn_pos_std     = 0.60;  ///< Respawn a mirror when the lone component's
                                                ///< horizontal position std exceeds this [m]
        };

        HypothesisParameters hyp;

        /**
         * @brief Activate the mixture with the current density and its 180 deg mirror.
         *
         * Two equal weights; later asymmetric evidence prunes the wrong one.
         */
        void initialise_hypotheses();

        /**
         * @brief Add the 180 deg mirror of the maximum-weight component as a new,
         *        equally weighted hypothesis (used for symmetry-flip recovery).
         */
        void spawn_mirror();

        /**
         * @brief Fold one frame of out-of-field side evidence into the mixture weights.
         *
         * @p log_ratio (FrameResult::side_delta) is added to the representative's log-weight.
         *
         * No-op unless at least two hypotheses are live.
         *
         * @param log_ratio Log-likelihood ratio own-vs-mirror for this frame [nats]
         */
        void add_side_log_evidence(double log_ratio);

        /**
         * @brief Process an event across every active hypothesis.
         *
         * Applies the event to each component, adds measurement log-evidence to its weight,
         * then normalises, merges, prunes, respawns and picks the representative.
         *
         * With no active hypotheses this is exactly `event.process(*this)`.
         *
         * @param event The event to apply (typically a Measurement)
         */
        void process(Event& event);

        /**
         * @brief Number of live hypotheses (1 in single-hypothesis mode).
         */
        std::size_t num_hypotheses() const {
            return components_.empty() ? 1 : components_.size();
        }

        /**
         * @brief Normalised (sum-to-one) linear weights of the live hypotheses.
         */
        std::vector<double> hypothesis_weights() const;

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
         *   (x, y) -> (-x, -y),  q -> mirror_quat_map()*q,  z/cam-bias unchanged.
         */
        static Eigen::VectorXd mirror_state(const Eigen::VectorXd& x);

        /**
         * @brief The 180 deg field-symmetry mirror of a pose density.
         */
        static GaussianInfo<double> mirror_density(const GaussianInfo<double>& g);

    protected:
        bool diffusing_ = false;  ///< Elevated fall PSDs in force (bounded window, see set_posture)

        std::vector<GaussianInfo<double>> components_;  ///< Mixture components (empty => single-hypothesis)
        std::vector<double> log_weights_;               ///< Unnormalised log weights per component
        Eigen::VectorXd last_rep_mean_;  ///< Outgoing representative mean (set_representative hysteresis)

        void normalise_weights();       ///< Renormalise log_weights_ (subtract log-sum-exp)
        void merge_components();        ///< Merge components within the merge gate (keep-best)
        void prune_components();        ///< Drop low-weight components; cap to max_components
        void respawn_if_unconfident();  ///< Respawn a mirror if collapsed to one uncertain component
        void set_representative();      ///< Set `density` to the maximum-weight component (tie-broken by hysteresis)
    };

}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_SYSTEMLOCALISATION_HPP
