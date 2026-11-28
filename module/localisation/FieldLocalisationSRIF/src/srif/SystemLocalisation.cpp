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
#include "SystemLocalisation.hpp"


namespace module::localisation::srif {
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::gaussian_filtering::measurement::Measurement;

    SystemLocalisation::SystemLocalisation(const GaussianInfo<double>& density) : SystemEstimator(density) {
        assert(density.dim() == nx);
    }

    SystemLocalisation* SystemLocalisation::clone() const {
        return new SystemLocalisation(*this);
    }

    // Rigid-body kinematics driven by the velocity states (templated for autodiff):
    //   d(rBFf)/dt = Rfb*vBb
    //   dq/dt      = 0.5*Xi(q)*omegaBb
    // Everything else is a random walk. vBb is modelled as nearly constant in the body frame,
    // so there is no Coriolis term.
    template <typename Scalar>
    static Eigen::VectorX<Scalar> dynamics_localisation_templated(const Eigen::VectorX<Scalar>& x) {
        assert(x.size() == SystemLocalisation::nx);

        const Eigen::Vector4<Scalar> q   = x.segment(SystemLocalisation::i_quat, 4);
        const Eigen::Matrix3<Scalar> Rfb = wxyz_to_mat(q);

        const Eigen::Vector3<Scalar> vBb     = x.segment(SystemLocalisation::i_vel, 3);
        const Eigen::Vector3<Scalar> omegaBb = x.segment(SystemLocalisation::i_omega, 3);

        Eigen::VectorX<Scalar> f(SystemLocalisation::nx);
        f.setZero();
        f.segment(SystemLocalisation::i_pos, 3) = Rfb * vBb;
        // Xi is linear in q, so there is no singularity at pitch = +-90 deg
        f.segment(SystemLocalisation::i_quat, 4) = Scalar(0.5) * wxyz_kinematics(q) * omegaBb;
        return f;
    }

    // The model is autonomous; u is unused but required by SystemBase.
    Eigen::VectorXd SystemLocalisation::dynamics(double /*t*/,
                                                 const Eigen::VectorXd& x,
                                                 const Eigen::VectorXd& /*u*/,
                                                 Eigen::MatrixXd& J) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::jacobian;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        auto func                  = [&](const Eigen::VectorX<dual>& xd) -> Eigen::VectorX<dual> {
            return dynamics_localisation_templated<dual>(xd);
        };

        Eigen::VectorX<dual> fdual;
        J = jacobian(func, wrt(xdual), at(xdual), fdual);

        Eigen::VectorXd f(x.size());
        for (Eigen::Index i = 0; i < f.size(); ++i) {
            f(i) = val(fdual(i));
        }
        return f;
    }

    Eigen::VectorXd SystemLocalisation::input(double, const Eigen::VectorXd&) const {
        // Autonomous: odometry and gyroscope enter as measurements of the velocity states
        return Eigen::VectorXd();
    }

    GaussianInfo<double> SystemLocalisation::process_noise_density(double dt) const {
        // dw ~ N^{-1}(0, LambdaQ/dt), i.e., cov(dw) = Q*dt
        Eigen::VectorXd sigma(nx);
        if (diffusing_) {
            // Hardware biases keep their ordinary PSDs; a fall does not change them
            sigma << params.sigma_pos_xy_disturbed, params.sigma_pos_xy_disturbed, params.sigma_pos_z_disturbed,
                quaternion_sigma(params.sigma_att_disturbed, params.sigma_yaw_disturbed),
                Eigen::Vector3d::Constant(params.sigma_vel_disturbed),
                Eigen::Vector3d::Constant(params.sigma_omega_disturbed),
                Eigen::Vector3d::Constant(params.sigma_gyro_bias), params.sigma_cam_bias, params.sigma_cam_bias;
        }
        else {
            sigma << params.sigma_pos_xy, params.sigma_pos_xy, params.sigma_pos_z,
                quaternion_sigma(params.sigma_att, params.sigma_yaw), Eigen::Vector3d::Constant(params.sigma_vel),
                Eigen::Vector3d::Constant(params.sigma_omega), Eigen::Vector3d::Constant(params.sigma_gyro_bias),
                params.sigma_cam_bias, params.sigma_cam_bias;
        }

        Eigen::MatrixXd XiQ = Eigen::MatrixXd::Zero(nx, nx);
        XiQ.diagonal()      = (sigma * std::sqrt(dt)).cwiseInverse();
        return GaussianInfo<double>::from_sqrt_info(XiQ);
    }

    std::vector<Eigen::Index> SystemLocalisation::process_noise_index() const {
        // Every state takes process noise; built from nx so it cannot disagree with
        // process_noise_density's dimension
        std::vector<Eigen::Index> idx(nx);
        std::iota(idx.begin(), idx.end(), Eigen::Index{0});
        return idx;
    }

    Eigen::Vector3d SystemLocalisation::body_velocity_from_odometry(const SensorsSample& a,
                                                                    const SensorsSample& b,
                                                                    double max_gap) {
        const Eigen::Vector3d invalid = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

        const double dt = b.t - a.t;
        if (dt <= 0 || dt > max_gap) {
            return invalid;
        }

        // Torso pose in world: Twt = Htw^{-1}
        const Eigen::Isometry3d Twta = a.Htw.inverse();
        const Eigen::Isometry3d Twtb = b.Htw.inverse();
        if (!Twta.translation().allFinite() || !Twtb.translation().allFinite() || !Twta.linear().allFinite()
            || !Twtb.linear().allFinite()) {
            return invalid;
        }

        // The relative pose is already a body-frame increment
        const Eigen::Vector3d vBb = (Twta.inverse() * Twtb).translation() / dt;
        return vBb.allFinite() ? vBb : invalid;
    }

    void SystemLocalisation::reset_to(const GaussianInfo<double>& new_density, double time) {
        assert(new_density.dim() == nx);
        density = new_density;
        time_   = time;
        // Collapses any mixture; callers re-seed hypotheses afterwards if wanted
        components_.clear();
        log_weights_.clear();
        last_rep_mean_ = Eigen::VectorXd();
    }

    void SystemLocalisation::inflate_covariance(const Eigen::VectorXd& extra_var) {
        assert(extra_var.size() == nx);
        assert((extra_var.array() >= 0.0).all());
        inflate_covariance(Eigen::MatrixXd(extra_var.asDiagonal()));
    }

    void SystemLocalisation::inflate_covariance(const Eigen::MatrixXd& extra_cov) {
        assert(extra_cov.rows() == nx && extra_cov.cols() == nx);

        auto inflate = [&](const GaussianInfo<double>& g) {
            Eigen::MatrixXd P = g.cov();
            P += extra_cov;
            // Symmetrise before the Cholesky in from_moment
            P = 0.5 * (P + P.transpose()).eval();
            return GaussianInfo<double>::from_moment(g.mean(), P);
        };

        if (components_.empty()) {
            density = inflate(density);
            return;
        }
        for (GaussianInfo<double>& c : components_) {
            c = inflate(c);
        }
        // With the bank live, `density` is a copy of the representative component
        set_representative();
    }

    void SystemLocalisation::predict_all(double time) {
        if (components_.empty()) {
            predict(time);
            return;
        }
        // Rewind the shared clock so each component predicts over the same interval
        const double t0 = time_;
        for (std::size_t i = 0; i < components_.size(); ++i) {
            time_   = t0;
            density = components_[i];
            predict(time);
            components_[i] = density;
        }
        set_representative();
    }

    // Project one density's attitude mean back onto the unit sphere.
    static GaussianInfo<double> project_quaternion(const GaussianInfo<double>& g) {
        Eigen::VectorXd mu = g.mean();
        const double n     = mu.segment<4>(SystemLocalisation::i_quat).norm();
        if (!(n > 1e-9)) {
            return g;  // Degenerate: leave it for the norm prior to pull back
        }
        mu.segment<4>(SystemLocalisation::i_quat) /= n;
        // q and -q are the same rotation; keep the mean in the w >= 0 hemisphere
        if (mu(SystemLocalisation::i_quat) < 0.0) {
            mu.segment<4>(SystemLocalisation::i_quat) = -mu.segment<4>(SystemLocalisation::i_quat);
        }
        return GaussianInfo<double>::from_moment(mu, g.cov());
    }

    void SystemLocalisation::predict(double time) {
        SystemEstimator::predict(time);
        // Integrating qdot drifts off the unit sphere at second order. Only `density` is
        // touched; predict_all uses it as the per-component working copy.
        density = project_quaternion(density);
    }

    void SystemLocalisation::normalise_quaternion() {
        // Projects the mean onto the unit sphere; MeasurementQuaternionNorm is only a soft prior.
        // The covariance is unchanged: to first order the projection is the identity on the
        // attitude directions.
        if (components_.empty()) {
            density = project_quaternion(density);
            return;
        }
        for (GaussianInfo<double>& c : components_) {
            c = project_quaternion(c);
        }
        set_representative();
    }

    // =========================================================================
    // Hypothesis bank
    // =========================================================================

    Eigen::VectorXd SystemLocalisation::mirror_state(const Eigen::VectorXd& x) {
        assert(x.size() == nx);
        // Premultiply the pose by Rz(pi) about the field centre: negate x and y, and apply
        // qz(pi) (x) q, i.e. (w,x,y,z) -> (-z,-y,x,w). Body-fixed velocities, rates and biases
        // are invariant.
        Eigen::VectorXd y    = x;
        y(0)                 = -x(0);
        y(1)                 = -x(1);
        y.segment<4>(i_quat) = mirror_quat_map() * Eigen::Vector4d(x.segment<4>(i_quat));
        return y;
    }

    GaussianInfo<double> SystemLocalisation::mirror_density(const GaussianInfo<double>& g) {
        assert(g.dim() == nx);
        // The mirror is linear, y = M*x with M orthogonal, so P' = M*P*M^T is exact
        Eigen::MatrixXd M             = Eigen::MatrixXd::Identity(nx, nx);
        M(0, 0)                       = -1.0;
        M(1, 1)                       = -1.0;
        M.block<4, 4>(i_quat, i_quat) = mirror_quat_map();

        Eigen::VectorXd mu = mirror_state(g.mean());
        Eigen::MatrixXd P  = g.cov();
        Eigen::MatrixXd Pm = M * P * M.transpose();
        // Symmetrise to remove any tiny asymmetry before the Cholesky in from_moment
        Pm = 0.5 * (Pm + Pm.transpose()).eval();
        return GaussianInfo<double>::from_moment(mu, Pm);
    }

    void SystemLocalisation::initialise_hypotheses() {
        components_.clear();
        log_weights_.clear();
        components_.push_back(density);
        components_.push_back(mirror_density(density));
        log_weights_.assign(components_.size(), 0.0);  // Equal weights
        last_rep_mean_ = Eigen::VectorXd();            // Fresh: representative is the seeded primary (index 0)
        set_representative();
    }

    void SystemLocalisation::spawn_mirror() {
        if (components_.empty()) {
            initialise_hypotheses();
            return;
        }
        // Add the leader's mirror at equal weight
        std::size_t best =
            std::distance(log_weights_.begin(), std::max_element(log_weights_.begin(), log_weights_.end()));
        components_.push_back(mirror_density(components_[best]));
        log_weights_.push_back(log_weights_[best]);
        normalise_weights();
        prune_components();
        set_representative();
    }

    void SystemLocalisation::add_side_log_evidence(double log_ratio) {
        if (components_.size() < 2 || !std::isfinite(log_ratio)) {
            return;  // Nothing to disambiguate, or no usable evidence this frame
        }
        // Favour the representative (the pose SideDisambiguator scored as "own") by log_ratio.
        // Under a near-tie it is the hysteresis pick, not the argmax, so match on the mean.
        const Eigen::VectorXd rep = density.mean();
        std::size_t rep_idx       = 0;
        double best_dist          = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < components_.size(); ++i) {
            const double d = (components_[i].mean() - rep).norm();
            if (d < best_dist) {
                best_dist = d;
                rep_idx   = i;
            }
        }
        log_weights_[rep_idx] += log_ratio;
        normalise_weights();
        prune_components();  // A decisive ratio drops the mirror below min_weight
        set_representative();
    }

    std::vector<double> SystemLocalisation::hypothesis_weights() const {
        if (components_.empty()) {
            return {1.0};
        }
        const double log_z =
            std::log(std::accumulate(
                log_weights_.begin(),
                log_weights_.end(),
                0.0,
                [max_log = *std::max_element(log_weights_.begin(), log_weights_.end())](double acc, double lw) {
                    return acc + std::exp(lw - max_log);
                }))
            + *std::max_element(log_weights_.begin(), log_weights_.end());
        std::vector<double> w(components_.size());
        for (std::size_t i = 0; i < components_.size(); ++i) {
            w[i] = std::exp(log_weights_[i] - log_z);
        }
        return w;
    }

    void SystemLocalisation::normalise_weights() {
        if (log_weights_.empty())
            return;
        const double max_log = *std::max_element(log_weights_.begin(), log_weights_.end());
        double sum           = 0.0;
        for (double lw : log_weights_)
            sum += std::exp(lw - max_log);
        const double log_z = max_log + std::log(sum);
        for (double& lw : log_weights_)
            lw -= log_z;  // Now sum(exp(log_weights_)) == 1
    }

    void SystemLocalisation::merge_components() {
        // Greedy keep-best merge of components within the position and yaw gates, summing
        // their weights
        bool merged = true;
        while (merged && components_.size() > 1) {
            merged = false;
            for (std::size_t i = 0; i < components_.size() && !merged; ++i) {
                for (std::size_t j = i + 1; j < components_.size(); ++j) {
                    Eigen::VectorXd mi = components_[i].mean();
                    Eigen::VectorXd mj = components_[j].mean();
                    double d_pos       = (mi.head<2>() - mj.head<2>()).norm();
                    double d_yaw       = std::abs(std::remainder(heading(mi) - heading(mj), 2.0 * M_PI));
                    if (d_pos < hyp.merge_position && d_yaw < hyp.merge_yaw) {
                        std::size_t keep = log_weights_[i] >= log_weights_[j] ? i : j;
                        std::size_t drop = keep == i ? j : i;
                        double a = log_weights_[i], b = log_weights_[j];
                        double m           = std::max(a, b);
                        double combined    = m + std::log(std::exp(a - m) + std::exp(b - m));
                        log_weights_[keep] = combined;
                        components_.erase(components_.begin() + drop);
                        log_weights_.erase(log_weights_.begin() + drop);
                        merged = true;
                        break;
                    }
                }
            }
        }
    }

    void SystemLocalisation::prune_components() {
        if (components_.size() <= 1)
            return;

        normalise_weights();
        std::vector<double> w = hypothesis_weights();

        // Drop components below the minimum normalised weight, always keeping the
        // single strongest hypothesis.
        std::size_t best = std::distance(w.begin(), std::max_element(w.begin(), w.end()));
        std::vector<GaussianInfo<double>> keep_c;
        std::vector<double> keep_lw;
        for (std::size_t i = 0; i < components_.size(); ++i) {
            if (i == best || w[i] >= hyp.min_weight) {
                keep_c.push_back(components_[i]);
                keep_lw.push_back(log_weights_[i]);
            }
        }
        components_.swap(keep_c);
        log_weights_.swap(keep_lw);

        // Cap to the strongest max_components hypotheses.
        if (components_.size() > hyp.max_components) {
            std::vector<std::size_t> order(components_.size());
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
                return log_weights_[a] > log_weights_[b];
            });
            std::vector<GaussianInfo<double>> top_c;
            std::vector<double> top_lw;
            for (std::size_t r = 0; r < hyp.max_components; ++r) {
                top_c.push_back(components_[order[r]]);
                top_lw.push_back(log_weights_[order[r]]);
            }
            components_.swap(top_c);
            log_weights_.swap(top_lw);
        }
        normalise_weights();
    }

    void SystemLocalisation::respawn_if_unconfident() {
        // A lone hypothesis cannot recover from a symmetry flip; re-seed its mirror while it is
        // uncertain in position
        if (components_.size() != 1)
            return;

        Eigen::MatrixXd P = components_.front().cov();
        double pos_std    = std::sqrt(std::max(P(0, 0), P(1, 1)));
        if (pos_std > hyp.respawn_pos_std) {
            components_.push_back(mirror_density(components_.front()));
            log_weights_.push_back(log_weights_.front());
            normalise_weights();
        }
    }

    void SystemLocalisation::set_representative() {
        if (components_.empty())
            return;
        const double max_lw = *std::max_element(log_weights_.begin(), log_weights_.end());

        // Hysteresis: symmetric hypotheses tie on landmark evidence, so among components within
        // tie_eps of the top weight keep the one nearest the previous representative
        constexpr double tie_eps = 0.5;  ///< Weight lead [nats] needed to unseat the incumbent
        std::size_t best         = 0;
        if (last_rep_mean_.size() == nx) {
            double best_dist = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < components_.size(); ++i) {
                if (log_weights_[i] >= max_lw - tie_eps) {
                    const double d = (components_[i].mean() - last_rep_mean_).norm();
                    if (d < best_dist) {
                        best_dist = d;
                        best      = i;
                    }
                }
            }
        }
        else {
            best = std::distance(log_weights_.begin(), std::max_element(log_weights_.begin(), log_weights_.end()));
        }
        density        = components_[best];
        last_rep_mean_ = density.mean();
    }

    void SystemLocalisation::process(Event& event) {
        // Single-hypothesis mode: ordinary single-Gaussian event processing.
        if (components_.empty()) {
            event.process(*this);
            normalise_quaternion();
            return;
        }

        // Multi-hypothesis mode: apply the event to each component, rewinding the shared clock
        const double t0   = time_;
        Measurement* meas = dynamic_cast<Measurement*>(&event);

        for (std::size_t i = 0; i < components_.size(); ++i) {
            time_   = t0;
            density = components_[i];
            // Score each hypothesis on its own data association
            if (meas != nullptr) {
                meas->reassociate(*this);
            }
            event.process(*this);
            components_[i] = density;
            if (meas) {
                double le = meas->log_evidence();
                if (std::isfinite(le)) {
                    log_weights_[i] += le;
                }
            }
        }
        normalise_quaternion();
        normalise_weights();
        merge_components();
        prune_components();
        respawn_if_unconfident();
        set_representative();

        // Leave the stored association matching the reported pose, for diagnostics
        if (meas != nullptr) {
            meas->reassociate(*this);
        }
    }
}  // namespace module::localisation::srif
