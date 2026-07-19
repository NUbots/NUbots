#include "SystemLocalisation.hpp"


namespace utility::slam::system {

    using utility::slam::camera::Pose;
    using utility::slam::gaussian::GaussianInfo;
    using utility::slam::measurement::Measurement;

    SystemLocalisation::SystemLocalisation(const GaussianInfo<double>& density,
                                           const std::vector<BodyTwistSample>& twistBuffer)
        : SystemEstimator(density), twistBuffer_(&twistBuffer) {
        assert(density.dim() == nx);
    }

    SystemLocalisation* SystemLocalisation::clone() const {
        return new SystemLocalisation(*this);
    }

    // Templated dynamics for autodiff:
    // deta/dt = JK(eta) * nu(t), JK(eta) = blkdiag(Rfb, TK(Theta))
    template <typename Scalar>
    static Eigen::VectorX<Scalar> dynamicsLocalisationTemplated(const Eigen::VectorX<Scalar>& x,
                                                                const Eigen::VectorXd& u) {
        assert(x.size() == SystemLocalisation::nx);
        assert(u.size() == 6);

        const Eigen::VectorX<Scalar> Theta = x.segment(3, 3);
        const Eigen::Matrix3<Scalar> Rfb   = rpy2rot(Theta);
        const Eigen::Matrix3<Scalar> TK    = TKfromThetaTemplated<Scalar>(Theta);

        const Eigen::Vector3d vBb     = u.head<3>();
        const Eigen::Vector3d omegaBb = u.tail<3>();

        Eigen::VectorX<Scalar> f(SystemLocalisation::nx);
        f.setZero();
        f.segment(0, 3) = Rfb * vBb.cast<Scalar>();
        f.segment(3, 3) = TK * omegaBb.cast<Scalar>();
        // Camera mount bias states are a random walk: zero drift
        return f;
    }

    Eigen::VectorXd SystemLocalisation::dynamics(double t, const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
        return dynamicsLocalisationTemplated<double>(x, u);
    }

    Eigen::VectorXd SystemLocalisation::dynamics(double t,
                                                 const Eigen::VectorXd& x,
                                                 const Eigen::VectorXd& u,
                                                 Eigen::MatrixXd& J) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::jacobian;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        auto func                  = [&](const Eigen::VectorX<dual>& xd) -> Eigen::VectorX<dual> {
            return dynamicsLocalisationTemplated<dual>(xd, u);
        };

        Eigen::VectorX<dual> fdual;
        J = jacobian(func, wrt(xdual), at(xdual), fdual);

        Eigen::VectorXd f(x.size());
        for (Eigen::Index i = 0; i < f.size(); ++i) {
            f(i) = val(fdual(i));
        }
        return f;
    }

    Eigen::VectorXd SystemLocalisation::input(double t, const Eigen::VectorXd& x) const {
        // Zero-order-hold lookup of the body twist at time t (zero outside buffer)
        Eigen::VectorXd u = Eigen::VectorXd::Zero(6);
        if (twistBuffer_ == nullptr || twistBuffer_->empty()) {
            return u;
        }

        const auto& buf = *twistBuffer_;
        auto it         = std::upper_bound(buf.begin(), buf.end(), t, [](double time, const BodyTwistSample& s) {
            return time < s.t;
        });
        if (it == buf.begin()) {
            return u;  // Before first sample
        }
        const BodyTwistSample& s = *std::prev(it);
        u.head<3>()              = s.vBb;
        u.tail<3>()              = s.omegaBb;
        return u;
    }

    GaussianInfo<double> SystemLocalisation::processNoiseDensity(double dt) const {
        // dw ~ N^{-1}(0, LambdaQ/dt), i.e., cov(dw) = Q*dt
        Eigen::VectorXd sigma(nx);
        sigma << params.sigmaPosXY, params.sigmaPosXY, params.sigmaPosZ, params.sigmaAtt, params.sigmaAtt,
            params.sigmaYaw, params.sigmaCamBias, params.sigmaCamBias;

        Eigen::MatrixXd XiQ = Eigen::MatrixXd::Zero(nx, nx);
        XiQ.diagonal()      = (sigma * std::sqrt(dt)).cwiseInverse();
        return GaussianInfo<double>::fromSqrtInfo(XiQ);
    }

    std::vector<Eigen::Index> SystemLocalisation::processNoiseIndex() const {
        return {0, 1, 2, 3, 4, 5, 6, 7};
    }

    std::vector<BodyTwistSample> SystemLocalisation::twistFromOdometry(const std::vector<SensorsSample>& sensors,
                                                                       double t0,
                                                                       double maxGap) {
        std::vector<BodyTwistSample> twists;
        if (sensors.size() < 2) {
            return twists;
        }
        twists.reserve(sensors.size() - 1);

        // Gyroscope bias from quiet samples. While the robot stands still the
        // gyro reads its bias directly (|omega| well below the walking
        // oscillation), so the mean over quiet samples calibrates it. The data2
        // recording shows ~0.75 deg/s of uncorrected z-bias, which otherwise
        // becomes a steady yaw drift the vision must keep fighting.
        Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();
        {
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            std::size_t n       = 0;
            for (const SensorsSample& s : sensors) {
                if (s.gyroscope.allFinite() && s.gyroscope.norm() < 0.05) {
                    sum += s.gyroscope;
                    n++;
                }
            }
            if (n >= 100) {
                gyroBias = sum / static_cast<double>(n);
            }
        }

        for (std::size_t k = 1; k < sensors.size(); ++k) {
            const SensorsSample& a = sensors[k - 1];
            const SensorsSample& b = sensors[k];
            const double dt        = b.t - a.t;
            if (dt <= 0 || dt > maxGap) {
                continue;
            }

            // Torso pose in world: Twt = Htw^{-1}
            Pose<double> Twta = Pose<double>(a.Htw.rotationMatrix, a.Htw.translationVector).inverse();
            Pose<double> Twtb = Pose<double>(b.Htw.rotationMatrix, b.Htw.translationVector).inverse();

            if (!Twta.translationVector.allFinite() || !Twtb.translationVector.allFinite()
                || !Twta.rotationMatrix.allFinite() || !Twtb.rotationMatrix.allFinite()) {
                continue;
            }

            // Relative pose of torso(b) w.r.t. torso(a): body-frame increment
            Pose<double> dT = Twta.inverse() * Twtb;

            Eigen::AngleAxisd aa(dT.rotationMatrix);

            BodyTwistSample s;
            s.t       = 0.5 * (a.t + b.t) - t0;
            s.vBb     = dT.translationVector / dt;
            s.omegaBb = aa.angle() * aa.axis() / dt;
            // The walk-engine odometry attitude slips badly while turning (data2
            // mocap: ~150 deg of yaw lost by t=40 s on odometry alone), but the
            // torso-frame gyroscope measures the angular velocity directly and is
            // immune to foot slip: prefer it whenever the sample carries one.
            const Eigen::Vector3d gyro = 0.5 * (a.gyroscope + b.gyroscope) - gyroBias;
            if (gyro.allFinite()) {
                s.omegaBb = gyro;
            }
            if (!s.vBb.allFinite() || !s.omegaBb.allFinite()) {
                continue;
            }
            twists.push_back(s);
        }
        return twists;
    }

    void SystemLocalisation::resetTo(const GaussianInfo<double>& newDensity, double time) {
        assert(newDensity.dim() == nx);
        density = newDensity;
        time_   = time;
    }

    GaussianInfo<double> SystemLocalisation::positionDensity() const {
        return density.marginal(Eigen::seqN(0, 3));
    }

    GaussianInfo<double> SystemLocalisation::orientationDensity() const {
        return density.marginal(Eigen::seqN(3, 3));
    }

    // =========================================================================
    // Hypothesis bank
    // =========================================================================

    Eigen::VectorXd SystemLocalisation::mirrorState(const Eigen::VectorXd& x) {
        assert(x.size() == nx);
        // 180 deg rotation about the field-centre z axis: premultiplying the pose
        // by Rz(pi) negates the horizontal position and offsets yaw by pi, leaving
        // roll, pitch, height and the camera-mount bias unchanged.
        Eigen::VectorXd y = x;
        y(0)              = -x(0);
        y(1)              = -x(1);
        y(5)              = std::remainder(x(5) + M_PI, 2.0 * M_PI);
        return y;
    }

    GaussianInfo<double> SystemLocalisation::mirrorDensity(const GaussianInfo<double>& g) {
        assert(g.dim() == nx);
        // The mirror map is affine: y = M*x + c with M = diag(-1,-1,1,1,1,1,1,1)
        // (the yaw offset pi lives in c and does not affect the covariance).
        // Hence mu' = mirrorState(mu) and P' = M*P*M^T, which for a diagonal sign
        // matrix simply flips the sign of the cross-covariances between the negated
        // (x, y) block and the rest.
        Eigen::VectorXd m = Eigen::VectorXd::Ones(nx);
        m(0)              = -1.0;
        m(1)              = -1.0;

        Eigen::VectorXd mu = mirrorState(g.mean());
        Eigen::MatrixXd P  = g.cov();
        Eigen::MatrixXd Pm = m.asDiagonal() * P * m.asDiagonal();
        // Symmetrise to remove any tiny asymmetry before the Cholesky in fromMoment
        Pm = 0.5 * (Pm + Pm.transpose()).eval();
        return GaussianInfo<double>::fromMoment(mu, Pm);
    }

    void SystemLocalisation::initialiseHypotheses() {
        components_.clear();
        logWeights_.clear();
        components_.push_back(density);
        components_.push_back(mirrorDensity(density));
        logWeights_.assign(components_.size(), 0.0);  // Equal weights
        setRepresentative();
    }

    void SystemLocalisation::spawnMirror() {
        if (components_.empty()) {
            initialiseHypotheses();
            return;
        }
        // Mirror the current leading hypothesis and add it at equal weight to the
        // leader, so a symmetry flip can be recovered from.
        std::size_t best = std::distance(logWeights_.begin(), std::max_element(logWeights_.begin(), logWeights_.end()));
        components_.push_back(mirrorDensity(components_[best]));
        logWeights_.push_back(logWeights_[best]);
        normaliseWeights();
        pruneComponents();
        setRepresentative();
    }

    std::vector<double> SystemLocalisation::hypothesisWeights() const {
        if (components_.empty()) {
            return {1.0};
        }
        const double logZ =
            std::log(std::accumulate(
                logWeights_.begin(),
                logWeights_.end(),
                0.0,
                [maxLog = *std::max_element(logWeights_.begin(), logWeights_.end())](double acc, double lw) {
                    return acc + std::exp(lw - maxLog);
                }))
            + *std::max_element(logWeights_.begin(), logWeights_.end());
        std::vector<double> w(components_.size());
        for (std::size_t i = 0; i < components_.size(); ++i) {
            w[i] = std::exp(logWeights_[i] - logZ);
        }
        return w;
    }

    void SystemLocalisation::normaliseWeights() {
        if (logWeights_.empty())
            return;
        const double maxLog = *std::max_element(logWeights_.begin(), logWeights_.end());
        double sum          = 0.0;
        for (double lw : logWeights_)
            sum += std::exp(lw - maxLog);
        const double logZ = maxLog + std::log(sum);
        for (double& lw : logWeights_)
            lw -= logZ;  // Now sum(exp(logWeights_)) == 1
    }

    void SystemLocalisation::mergeComponents() {
        // Greedy keep-best merge: components whose means lie within the position and
        // yaw gates are collapsed, retaining the higher-weight density and summing
        // their weights (log-sum-exp). Symmetric field hypotheses sit a whole field
        // apart, so this only fuses genuine duplicates.
        bool merged = true;
        while (merged && components_.size() > 1) {
            merged = false;
            for (std::size_t i = 0; i < components_.size() && !merged; ++i) {
                for (std::size_t j = i + 1; j < components_.size(); ++j) {
                    Eigen::VectorXd mi = components_[i].mean();
                    Eigen::VectorXd mj = components_[j].mean();
                    double dPos        = (mi.head<2>() - mj.head<2>()).norm();
                    double dYaw        = std::abs(std::remainder(mi(5) - mj(5), 2.0 * M_PI));
                    if (dPos < hyp.mergePosition && dYaw < hyp.mergeYaw) {
                        std::size_t keep = logWeights_[i] >= logWeights_[j] ? i : j;
                        std::size_t drop = keep == i ? j : i;
                        double a = logWeights_[i], b = logWeights_[j];
                        double m          = std::max(a, b);
                        double combined   = m + std::log(std::exp(a - m) + std::exp(b - m));
                        components_[keep] = components_[keep];  // density unchanged (keep-best)
                        logWeights_[keep] = combined;
                        components_.erase(components_.begin() + drop);
                        logWeights_.erase(logWeights_.begin() + drop);
                        merged = true;
                        break;
                    }
                }
            }
        }
    }

    void SystemLocalisation::pruneComponents() {
        if (components_.size() <= 1)
            return;

        normaliseWeights();
        std::vector<double> w = hypothesisWeights();

        // Drop components below the minimum normalised weight, always keeping the
        // single strongest hypothesis.
        std::size_t best = std::distance(w.begin(), std::max_element(w.begin(), w.end()));
        std::vector<GaussianInfo<double>> keepC;
        std::vector<double> keepLW;
        for (std::size_t i = 0; i < components_.size(); ++i) {
            if (i == best || w[i] >= hyp.minWeight) {
                keepC.push_back(components_[i]);
                keepLW.push_back(logWeights_[i]);
            }
        }
        components_.swap(keepC);
        logWeights_.swap(keepLW);

        // Cap to the strongest maxComponents hypotheses.
        if (components_.size() > hyp.maxComponents) {
            std::vector<std::size_t> order(components_.size());
            std::iota(order.begin(), order.end(), 0);
            std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
                return logWeights_[a] > logWeights_[b];
            });
            std::vector<GaussianInfo<double>> topC;
            std::vector<double> topLW;
            for (std::size_t r = 0; r < hyp.maxComponents; ++r) {
                topC.push_back(components_[order[r]]);
                topLW.push_back(logWeights_[order[r]]);
            }
            components_.swap(topC);
            logWeights_.swap(topLW);
        }
        normaliseWeights();
    }

    void SystemLocalisation::respawnIfUnconfident() {
        // Once the bank has collapsed to a single hypothesis it can no longer
        // recover from a symmetry flip. If that lone hypothesis is spatially
        // uncertain (large horizontal position std), re-seed its mirror so the
        // symmetry can be re-resolved by future evidence.
        if (components_.size() != 1)
            return;

        Eigen::MatrixXd P = components_.front().cov();
        double posStd     = std::sqrt(std::max(P(0, 0), P(1, 1)));
        if (posStd > hyp.respawnPosStd) {
            components_.push_back(mirrorDensity(components_.front()));
            logWeights_.push_back(logWeights_.front());
            normaliseWeights();
        }
    }

    void SystemLocalisation::setRepresentative() {
        if (components_.empty())
            return;
        std::size_t best = std::distance(logWeights_.begin(), std::max_element(logWeights_.begin(), logWeights_.end()));
        density          = components_[best];
    }

    void SystemLocalisation::process(Event& event) {
        // Single-hypothesis mode: ordinary single-Gaussian event processing.
        if (components_.empty()) {
            event.process(*this);
            return;
        }

        // Multi-hypothesis mode: apply the event to each component through the same
        // verified predict/update path, rewinding the shared system clock each time
        // so every component predicts over the identical interval.
        const double t0   = time_;
        Measurement* meas = dynamic_cast<Measurement*>(&event);

        for (std::size_t i = 0; i < components_.size(); ++i) {
            time_   = t0;
            density = components_[i];
            event.process(*this);
            components_[i] = density;
            if (meas) {
                double le = meas->logEvidence();
                if (std::isfinite(le)) {
                    logWeights_[i] += le;
                }
            }
        }
        // time_ is now the event time (set by the last component's predict).

        normaliseWeights();
        mergeComponents();
        pruneComponents();
        respawnIfUnconfident();
        setRepresentative();
    }
}  // namespace utility::slam::system
