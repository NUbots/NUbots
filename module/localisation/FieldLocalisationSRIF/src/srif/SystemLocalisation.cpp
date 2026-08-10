#include "SystemLocalisation.hpp"


namespace module::localisation::srif {

    using utility::gaussian_filtering::Pose;
    using utility::gaussian_filtering::gaussian::GaussianInfo;
    using utility::gaussian_filtering::measurement::Measurement;

    SystemLocalisation::SystemLocalisation(const GaussianInfo<double>& density) : SystemEstimator(density) {
        assert(density.dim() == nx);
    }

    SystemLocalisation* SystemLocalisation::clone() const {
        return new SystemLocalisation(*this);
    }

    // Templated dynamics for autodiff. Autonomous: nothing is a known input any more,
    // so the derivative is the rigid-body kinematics driven by the velocity STATES.
    //
    //   d(rBFf)/dt = Rfb*vBb           (Fossen's eta-dot = J(eta)*nu, position half)
    //   dq/dt      = 0.5*Xi(q)*omegaBb (        ... and the attitude half)
    //   everything else is a random walk: zero drift, process noise only
    //
    // Note there is no Coriolis term in d(vBb)/dt. That would appear if vBb were
    // driven by measured specific force (a strapdown INS); here the model is simply
    // that the body-fixed velocity is nearly constant between measurements, which is
    // exactly why body-fixed is the right frame to carry it in -- a robot walking a
    // curve holds vBb while its field-frame velocity rotates the whole way round.
    template <typename Scalar>
    static Eigen::VectorX<Scalar> dynamicsLocalisationTemplated(const Eigen::VectorX<Scalar>& x) {
        assert(x.size() == SystemLocalisation::nx);

        const Eigen::Vector4<Scalar> q   = x.segment(SystemLocalisation::iQuat, 4);
        const Eigen::Matrix3<Scalar> Rfb = quat2rot(q);

        const Eigen::Vector3<Scalar> vBb     = x.segment(SystemLocalisation::iVel, 3);
        const Eigen::Vector3<Scalar> omegaBb = x.segment(SystemLocalisation::iOmega, 3);

        Eigen::VectorX<Scalar> f(SystemLocalisation::nx);
        f.setZero();
        f.segment(SystemLocalisation::iPos, 3) = Rfb * vBb;
        // qdot = 0.5*Xi(q)*omega_b. No singularity anywhere: every entry of Xi is
        // linear in q, so the toppling robot that used to blow up the roll-pitch-yaw
        // rate transform at pitch = +-90 deg now integrates like any other attitude.
        f.segment(SystemLocalisation::iQuat, 4) = Scalar(0.5) * quatXi(q) * omegaBb;
        // vBb, omegaBb, the gyroscope bias and the camera mount bias are all random
        // walks: zero drift.
        return f;
    }

    // The input is unused: the process model is autonomous, driven by the velocity
    // states rather than by a twist fed in from outside. The parameter stays because
    // SystemBase's interface is shared with input-driven systems.
    Eigen::VectorXd SystemLocalisation::dynamics(double /*t*/,
                                                 const Eigen::VectorXd& x,
                                                 const Eigen::VectorXd& /*u*/) const {
        return dynamicsLocalisationTemplated<double>(x);
    }

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
            return dynamicsLocalisationTemplated<dual>(xd);
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
        // The process model is autonomous. What used to be a known input -- the body
        // twist finite-differenced from walk-engine odometry, with the gyroscope
        // substituted for its angular part -- is now two measurements
        // (MeasurementBodyVelocity, MeasurementGyroscope) of two velocity states. That
        // is the whole point of carrying the rates: the odometry's noise is modelled
        // where it belongs instead of being asserted as truth, and the gyroscope's bias
        // becomes an estimated state rather than a heuristic subtracted up front.
        return Eigen::VectorXd();
    }

    GaussianInfo<double> SystemLocalisation::processNoiseDensity(double dt) const {
        // dw ~ N^{-1}(0, LambdaQ/dt), i.e., cov(dw) = Q*dt
        Eigen::VectorXd sigma(nx);
        if (diffusing_) {
            // The gyroscope and camera-mount biases are properties of the hardware, not
            // of the posture, so they keep their ordinary (deliberately tiny) PSDs: a
            // fall is no reason to let a calibration wander.
            sigma << params.sigmaPosXYDisturbed, params.sigmaPosXYDisturbed, params.sigmaPosZDisturbed,
                quaternionSigma(params.sigmaAttDisturbed, params.sigmaYawDisturbed),
                Eigen::Vector3d::Constant(params.sigmaVelDisturbed),
                Eigen::Vector3d::Constant(params.sigmaOmegaDisturbed), Eigen::Vector3d::Constant(params.sigmaGyroBias),
                params.sigmaCamBias, params.sigmaCamBias;
        }
        else {
            sigma << params.sigmaPosXY, params.sigmaPosXY, params.sigmaPosZ,
                quaternionSigma(params.sigmaAtt, params.sigmaYaw), Eigen::Vector3d::Constant(params.sigmaVel),
                Eigen::Vector3d::Constant(params.sigmaOmega), Eigen::Vector3d::Constant(params.sigmaGyroBias),
                params.sigmaCamBias, params.sigmaCamBias;
        }

        Eigen::MatrixXd XiQ = Eigen::MatrixXd::Zero(nx, nx);
        XiQ.diagonal()      = (sigma * std::sqrt(dt)).cwiseInverse();
        return GaussianInfo<double>::fromSqrtInfo(XiQ);
    }

    std::vector<Eigen::Index> SystemLocalisation::processNoiseIndex() const {
        // Every state takes process noise. Built from nx rather than written out: the
        // list silently disagreeing with processNoiseDensity's dimension is not caught
        // anywhere, and when the state grew to carry a quaternion the stale eight-entry
        // literal turned the first non-zero-dt prediction into NaN.
        std::vector<Eigen::Index> idx(nx);
        std::iota(idx.begin(), idx.end(), Eigen::Index{0});
        return idx;
    }

    std::vector<BodyTwistSample> SystemLocalisation::twistFromOdometry(const std::vector<SensorsSample>& sensors,
                                                                       double t0,
                                                                       double maxGap) {
        std::vector<BodyTwistSample> twists;
        if (sensors.size() < 2) {
            return twists;
        }
        twists.reserve(sensors.size() - 1);

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
            s.t   = 0.5 * (a.t + b.t) - t0;
            s.vBb = dT.translationVector / dt;
            // Odometry-derived angular rate. Previously the gyroscope was substituted in
            // here, because the walk-engine odometry attitude slips badly while turning
            // (data2 mocap: ~150 deg of yaw lost by t=40 s on odometry alone). It is no
            // longer substituted: the gyroscope is its own measurement of omegaBb with
            // its own noise and its own estimated bias, and folding it into this sample
            // would feed the same reading in twice. This field is only useful as a
            // fallback when no gyroscope is available.
            s.omegaBb = aa.angle() * aa.axis() / dt;
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
        // A reset asserts a single known belief, so it collapses any mixture -- the
        // old components describe a state we are declaring void (init, injected
        // kidnap). Recovery of the alternative, if needed, is re-seeded afterwards
        // (initialiseHypotheses at startup, or spawnMirror on out-of-field doubt).
        components_.clear();
        logWeights_.clear();
        lastRepMean_ = Eigen::VectorXd();
    }

    void SystemLocalisation::inflateCovariance(const Eigen::VectorXd& extraVar) {
        assert(extraVar.size() == nx);
        assert((extraVar.array() >= 0.0).all());
        inflateCovariance(Eigen::MatrixXd(extraVar.asDiagonal()));
    }

    void SystemLocalisation::inflateCovariance(const Eigen::MatrixXd& extraCov) {
        assert(extraCov.rows() == nx && extraCov.cols() == nx);

        auto inflate = [&](const GaussianInfo<double>& g) {
            Eigen::MatrixXd P = g.cov();
            P += extraCov;
            // Symmetrise before the Cholesky in fromMoment, matching mirrorDensity.
            P = 0.5 * (P + P.transpose()).eval();
            return GaussianInfo<double>::fromMoment(g.mean(), P);
        };

        if (components_.empty()) {
            density = inflate(density);
            return;
        }
        for (GaussianInfo<double>& c : components_) {
            c = inflate(c);
        }
        // With the bank live, `density` is a copy of the representative component
        // rather than state in its own right, so it is refreshed from the inflated
        // components instead of being inflated separately. Inflation does not move
        // any mean, so the representative does not change.
        setRepresentative();
    }

    void SystemLocalisation::predictAll(double time) {
        if (components_.empty()) {
            predict(time);
            return;
        }
        // Rewind the shared clock per component so each predicts over the identical
        // interval, exactly as process() does.
        const double t0 = time_;
        for (std::size_t i = 0; i < components_.size(); ++i) {
            time_   = t0;
            density = components_[i];
            predict(time);
            components_[i] = density;
        }
        setRepresentative();
    }

    // Project one density's attitude mean back onto the unit sphere.
    static GaussianInfo<double> projectQuaternion(const GaussianInfo<double>& g) {
        Eigen::VectorXd mu = g.mean();
        const double n     = mu.segment<4>(SystemLocalisation::iQuat).norm();
        if (!(n > 1e-9)) {
            return g;  // Degenerate: leave it for the norm prior to pull back
        }
        mu.segment<4>(SystemLocalisation::iQuat) /= n;
        // q and -q are the same rotation. Letting the mean wander between the two
        // hemispheres would make a Gaussian over the components describe a bimodal
        // thing it cannot represent, so it is kept in w >= 0.
        if (mu(SystemLocalisation::iQuat) < 0.0) {
            mu.segment<4>(SystemLocalisation::iQuat) = -mu.segment<4>(SystemLocalisation::iQuat);
        }
        return GaussianInfo<double>::fromMoment(mu, g.cov());
    }

    void SystemLocalisation::predict(double time) {
        SystemEstimator::predict(time);
        // Prediction integrates qdot, which leaves the unit sphere at second order
        // over a step and would otherwise let |q| wander. Only `density` is touched:
        // predictAll drives this per component with density as its working copy.
        density = projectQuaternion(density);
    }

    void SystemLocalisation::normaliseQuaternion() {
        // Renormalising the mean is a projection back onto the unit sphere, which
        // MeasurementQuaternionNorm keeps the belief near but cannot enforce exactly
        // (it is a soft prior, and prediction pushes off the sphere between updates).
        // The covariance is left alone: to first order the projection's Jacobian is
        // the identity on the three attitude directions, and the radial direction is
        // the one the norm prior owns.
        if (components_.empty()) {
            density = projectQuaternion(density);
            return;
        }
        for (GaussianInfo<double>& c : components_) {
            c = projectQuaternion(c);
        }
        setRepresentative();
    }

    // =========================================================================
    // Hypothesis bank
    // =========================================================================

    Eigen::VectorXd SystemLocalisation::mirrorState(const Eigen::VectorXd& x) {
        assert(x.size() == nx);
        // 180 deg rotation about the field-centre z axis: premultiplying the pose by
        // Rz(pi) negates the horizontal position and turns the attitude by pi about
        // field z, leaving height and the camera-mount bias unchanged. On the
        // quaternion that premultiplication is qz(pi) (x) q with qz(pi) = (0,0,0,1),
        // i.e. (w,x,y,z) -> (-z,-y,x,w) -- a signed permutation, so unlike the yaw
        // offset it used to be it is exactly linear and needs no wrapping.
        //
        // The velocity states are untouched, and that is a property of carrying them
        // body-fixed rather than an oversight: v_b' = (Rz(pi) Rfb)^T Rz(pi) v_f = v_b.
        // The robot is doing exactly the same thing, just somewhere else on the field.
        // Field-frame velocity would have needed its horizontal components negated
        // alongside the position. The gyroscope and camera biases are hardware
        // properties and are likewise unaffected by where the robot is standing.
        Eigen::VectorXd y   = x;
        y(0)                = -x(0);
        y(1)                = -x(1);
        y.segment<4>(iQuat) = mirrorQuatMap() * Eigen::Vector4d(x.segment<4>(iQuat));
        return y;
    }

    GaussianInfo<double> SystemLocalisation::mirrorDensity(const GaussianInfo<double>& g) {
        assert(g.dim() == nx);
        // The mirror map is now exactly linear: y = M*x, with M block-diagonal over
        // (position, quaternion, camera bias). It used to be affine because the yaw
        // offset pi lived in the constant term; on the quaternion the same rotation
        // is the signed permutation mirrorQuatMap(), which is orthogonal, so
        // P' = M*P*M^T is exact rather than a small-angle approximation.
        Eigen::MatrixXd M           = Eigen::MatrixXd::Identity(nx, nx);
        M(0, 0)                     = -1.0;
        M(1, 1)                     = -1.0;
        M.block<4, 4>(iQuat, iQuat) = mirrorQuatMap();

        Eigen::VectorXd mu = mirrorState(g.mean());
        Eigen::MatrixXd P  = g.cov();
        Eigen::MatrixXd Pm = M * P * M.transpose();
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
        lastRepMean_ = Eigen::VectorXd();             // Fresh: representative is the seeded primary (index 0)
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

    void SystemLocalisation::addSideLogEvidence(double logRatio) {
        if (components_.size() < 2 || !std::isfinite(logRatio)) {
            return;  // Nothing to disambiguate, or no usable evidence this frame
        }
        // Favour the representative -- the pose SideDisambiguator scored as "own" --
        // by logRatio. It is the component matching the current density, which under
        // a near-tie is the hysteresis pick, NOT necessarily the max-weight one, so
        // match on the mean rather than taking an argmax. With the usual own+mirror
        // pair this is a shift of the pair; normaliseWeights keeps them summing to one.
        const Eigen::VectorXd rep = density.mean();
        std::size_t repIdx        = 0;
        double bestDist           = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < components_.size(); ++i) {
            const double d = (components_[i].mean() - rep).norm();
            if (d < bestDist) {
                bestDist = d;
                repIdx   = i;
            }
        }
        logWeights_[repIdx] += logRatio;
        normaliseWeights();
        pruneComponents();  // A decisive ratio drops the mirror below minWeight
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
                    // Through heading(), not an element: x(5) was yaw under the old
                    // roll-pitch-yaw state but is the quaternion's y component now.
                    double dYaw = std::abs(std::remainder(heading(mi) - heading(mj), 2.0 * M_PI));
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
        const double maxLW = *std::max_element(logWeights_.begin(), logWeights_.end());

        // Hysteresis on a near-tie: the two symmetric hypotheses sit at 50/50 on
        // landmark evidence alone and are numerically indistinguishable, so a plain
        // argmax would swap the reported pose between a side and its mirror on
        // floating-point noise. Among components within kTieEps of the top weight,
        // keep the one nearest the outgoing representative; a decisive lead
        // (out-of-field evidence) exceeds kTieEps and switches cleanly.
        constexpr double kTieEps = 0.5;  ///< Weight lead [nats] needed to unseat the incumbent
        std::size_t best         = 0;
        if (lastRepMean_.size() == nx) {
            double bestDist = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < components_.size(); ++i) {
                if (logWeights_[i] >= maxLW - kTieEps) {
                    const double d = (components_[i].mean() - lastRepMean_).norm();
                    if (d < bestDist) {
                        bestDist = d;
                        best     = i;
                    }
                }
            }
        }
        else {
            best = std::distance(logWeights_.begin(), std::max_element(logWeights_.begin(), logWeights_.end()));
        }
        density      = components_[best];
        lastRepMean_ = density.mean();
    }

    void SystemLocalisation::process(Event& event) {
        // Single-hypothesis mode: ordinary single-Gaussian event processing.
        if (components_.empty()) {
            event.process(*this);
            normaliseQuaternion();
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
            // Re-run any pose-dependent data association against THIS component
            // before the update, so a hypothesis is scored on its own assignment
            // rather than the representative's (no-op for measurements without
            // association). The single-hypothesis path above never calls this, so
            // its behaviour is byte-for-byte unchanged.
            if (meas != nullptr) {
                meas->reassociate(*this);
            }
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

        normaliseQuaternion();
        normaliseWeights();
        mergeComponents();
        pruneComponents();
        respawnIfUnconfident();
        setRepresentative();

        // Leave the measurement's stored association matching the representative, so
        // downstream diagnostics (numAssociated, the residual overlays) reflect the
        // pose actually reported rather than whichever component happened to be
        // processed last.
        if (meas != nullptr) {
            meas->reassociate(*this);
        }
    }
}  // namespace module::localisation::srif
