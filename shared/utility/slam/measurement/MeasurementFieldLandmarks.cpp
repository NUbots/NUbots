#include "MeasurementFieldLandmarks.hpp"

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/dual/eigen.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "../FieldMap.hpp"
#include "../measurement/Measurement.hpp"
#include "../system/SystemLocalisation.hpp"

namespace utility::slam::measurement {

    MeasurementFieldLandmarks::MeasurementFieldLandmarks(double time,
                                                         const VisionSample& sample,
                                                         const Pose<double>& Tbc,
                                                         const FieldMap& map,
                                                         const SystemLocalisation& system,
                                                         const Options& options)
        : Measurement(time), map_(map), Tbc_(Tbc), options_(options) {
        // Exact Hessian is cheap for a 6-dim state and yields the exact
        // Laplace-approximation posterior sqrt information
        updateMethod_ = UpdateMethod::NEWTONTRUSTEIG;

        // Gather usable detections
        for (std::size_t i = 0; i < sample.detections.size(); ++i) {
            const Detection& det = sample.detections[i];
            if (det.confidence < options_.minConfidence) {
                continue;
            }
            Eigen::Vector3d ray;
            LandmarkType type;
            if (detectionRay(det, ray, type)) {
                // Confidence sets how much of this detection's likelihood is the
                // inlier Gaussian rather than flat clutter (see Options).
                const double w = std::clamp(options_.inlierProbability * det.confidence / options_.confidenceReference,
                                            1e-3,
                                            options_.maxInlierProbability);
                candidates_.push_back({ray, type, i, w});
            }
        }

        assocKeys_ = associate(system.density.mean(), system.density.cov());
    }

    MeasurementFieldLandmarks::MeasurementFieldLandmarks(double time,
                                                         const VisionSample& sample,
                                                         const Pose<double>& Tbc,
                                                         const FieldMap& map,
                                                         const SystemLocalisation& system)
        : MeasurementFieldLandmarks(time, sample, Tbc, map, system, Options{}) {}

    bool MeasurementFieldLandmarks::detectionRay(const Detection& det, Eigen::Vector3d& ray, LandmarkType& type) {
        if (det.name == "L-intersection") {
            type = LandmarkType::L_INTERSECTION;
            ray  = det.corners.rowwise().sum();  // Bounding box centre
        }
        else if (det.name == "T-intersection") {
            type = LandmarkType::T_INTERSECTION;
            ray  = det.corners.rowwise().sum();
        }
        else if (det.name == "X-intersection") {
            type = LandmarkType::X_INTERSECTION;
            ray  = det.corners.rowwise().sum();
        }
        else if (det.name == "goal post") {
            type = LandmarkType::GOAL_POST;
            ray  = det.corners.col(2) + det.corners.col(3);  // Bottom-centre (post base): BR + BL
        }
        else {
            return false;  // ball, robot, etc. are not mapped landmarks
        }

        if (!ray.allFinite() || ray.norm() < 1e-12) {
            return false;
        }
        ray.normalize();
        return true;
    }

    std::vector<std::pair<std::size_t, std::size_t>> MeasurementFieldLandmarks::associate(const Eigen::VectorXd& x,
                                                                                          const Eigen::MatrixXd& P) {
        std::vector<std::pair<std::size_t, std::size_t>> keys;
        assocCand_.clear();
        inlierWeight_.clear();
        if (candidates_.empty()) {
            uMeas_.resize(3, 0);
            rLFf_.resize(3, 0);
            return keys;
        }

        // Predicted ray for every mapped landmark at the given state (incl. camera mount bias)
        Pose<double> Tbias(SystemLocalisation::cameraBiasRotation<double>(x), Eigen::Vector3d::Zero());
        Pose<double> Tfc           = SystemLocalisation::fieldPose<double>(x) * Tbc_ * Tbias;
        const Eigen::Matrix3d Rfc  = Tfc.rotationMatrix;
        const Eigen::Vector3d rCFf = Tfc.translationVector;

        // Pose uncertainty inflating each predicted bearing. Bearings are compared in
        // {f}: the tangent-plane geometry and the yaw term are both natural there.
        const Eigen::Matrix3d Ppos = P.topLeftCorner<3, 3>();
        const double yawVar        = P(5, 5);
        const double sigma2        = options_.sigmaAngular * options_.sigmaAngular;
        const double cosGate       = std::cos(options_.gateAngle);

        // SNN: enumerate all (detection, landmark) pairs of matching type inside the
        // geometric pre-gate, score each by its surprisal relative to the clutter
        // crossover, then assign best-scoring pairs first, each detection/landmark at
        // most once. A negative score means the inlier component explains the pair
        // better than clutter does.
        struct CandidatePair {
            double score;
            std::size_t det;
            LandmarkType type;
            std::size_t lm;
        };
        std::vector<CandidatePair> pairs;
        for (std::size_t i = 0; i < candidates_.size(); ++i) {
            const Eigen::Vector3d uMeasF = Rfc * candidates_[i].ray;
            // Crossover surprisal: w N(e; S) > (1 - w)/(4 pi). A weak detection has to
            // fit much better before it is worth associating at all, and ranks below a
            // confident one at equal residual, which is what stops the stealing.
            const double w         = candidates_[i].inlierWeight;
            const double crossover = std::log(w) - std::log(1.0 - w) + std::log(4.0 * M_PI);

            const std::vector<Eigen::Vector3d>& lms = map_.landmarks(candidates_[i].type);
            for (std::size_t j = 0; j < lms.size(); ++j) {
                const Eigen::Vector3d rel = lms[j] - rCFf;
                const double range        = rel.norm();
                if (range < 1e-9) {
                    continue;
                }
                const Eigen::Vector3d uPredF = rel / range;
                if (uMeasF.dot(uPredF) < cosGate) {
                    continue;  // Cheap geometric pre-gate: caps how far an association can reach
                }

                // Innovation covariance in the tangent plane: bearing noise + camera
                // position uncertainty across the range + yaw uncertainty.
                const Eigen::Matrix<double, 3, 2> T = tangentBasis(uPredF);
                const Eigen::Vector2d a             = T.transpose() * Eigen::Vector3d::UnitZ().cross(uPredF);
                const Eigen::Matrix2d S             = Eigen::Matrix2d::Identity() * sigma2
                                          + T.transpose() * Ppos * T / (range * range) + yawVar * a * a.transpose();
                const Eigen::Vector2d e = T.transpose() * (uMeasF - uPredF);
                const double surprisal =
                    0.5 * e.dot(S.inverse() * e) + 0.5 * std::log(S.determinant()) + std::log(2.0 * M_PI);
                const double score = surprisal - crossover;
                if (score < 0.0) {
                    pairs.push_back({score, i, candidates_[i].type, j});
                }
            }
        }
        std::sort(pairs.begin(), pairs.end(), [](const CandidatePair& a, const CandidatePair& b) {
            return a.score < b.score;
        });

        std::vector<bool> detUsed(candidates_.size(), false);
        // Landmark usage tracked per type via flat key: type-major index
        auto lmKey = [](LandmarkType type, std::size_t j) { return static_cast<std::size_t>(type) * 1000 + j; };
        std::vector<std::size_t> usedLandmarks;
        std::vector<const CandidatePair*> chosen;
        for (const CandidatePair& p : pairs) {
            if (detUsed[p.det]) {
                continue;
            }
            std::size_t key = lmKey(p.type, p.lm);
            if (std::find(usedLandmarks.begin(), usedLandmarks.end(), key) != usedLandmarks.end()) {
                continue;
            }
            detUsed[p.det] = true;
            usedLandmarks.push_back(key);
            chosen.push_back(&p);
        }

        uMeas_.resize(3, static_cast<Eigen::Index>(chosen.size()));
        rLFf_.resize(3, static_cast<Eigen::Index>(chosen.size()));
        assocCand_.reserve(chosen.size());
        inlierWeight_.reserve(chosen.size());
        for (std::size_t k = 0; k < chosen.size(); ++k) {
            const CandidatePair& p                   = *chosen[k];
            uMeas_.col(static_cast<Eigen::Index>(k)) = candidates_[p.det].ray;
            rLFf_.col(static_cast<Eigen::Index>(k))  = map_.landmarks(p.type)[p.lm];
            assocCand_.push_back(p.det);
            inlierWeight_.push_back(candidates_[p.det].inlierWeight);
            keys.emplace_back(p.det, lmKey(p.type, p.lm));
        }
        std::sort(keys.begin(), keys.end());
        return keys;
    }

    std::vector<MeasurementFieldLandmarks::DetectionOutcome> MeasurementFieldLandmarks::detectionOutcomes() const {
        std::vector<DetectionOutcome> outcomes;
        outcomes.reserve(candidates_.size());
        for (std::size_t i = 0; i < candidates_.size(); ++i) {
            const bool associated = std::find(assocCand_.begin(), assocCand_.end(), i) != assocCand_.end();
            outcomes.push_back({candidates_[i].detection, associated});
        }
        return outcomes;
    }

    Eigen::VectorXd MeasurementFieldLandmarks::simulate(const Eigen::VectorXd& x,
                                                        const SystemEstimator& /*system*/) const {
        // Stack predicted rays (noise-free measurement model)
        Eigen::Matrix<double, 3, Eigen::Dynamic> uPred = predictRays<double>(x);
        return Eigen::Map<Eigen::VectorXd>(uPred.data(), uPred.size());
    }

    double MeasurementFieldLandmarks::logLikelihood(const Eigen::VectorXd& x, const SystemEstimator& /*system*/) const {
        return logLikelihoodImpl<double>(x);
    }

    double MeasurementFieldLandmarks::logLikelihood(const Eigen::VectorXd& x,
                                                    const SystemEstimator& /*system*/,
                                                    Eigen::VectorXd& g) const {
        using autodiff::at;
        using autodiff::dual;
        using autodiff::gradient;
        using autodiff::wrt;

        Eigen::VectorX<dual> xdual = x.cast<dual>();
        dual fdual;
        auto func = [this](const Eigen::VectorX<dual>& xd) -> dual {
            return this->template logLikelihoodImpl<dual>(xd);
        };
        g = gradient(func, wrt(xdual), at(xdual), fdual);
        return static_cast<double>(fdual);
    }

    double MeasurementFieldLandmarks::logLikelihood(const Eigen::VectorXd& x,
                                                    const SystemEstimator& /*system*/,
                                                    Eigen::VectorXd& g,
                                                    Eigen::MatrixXd& H) const {
        using autodiff::at;
        using autodiff::dual2nd;
        using autodiff::hessian;
        using autodiff::wrt;

        g.resize(x.size());
        H.resize(x.size(), x.size());

        Eigen::VectorX<dual2nd> xdual = x.cast<dual2nd>();
        dual2nd fdual;
        auto func = [this](const Eigen::VectorX<dual2nd>& xd) -> dual2nd {
            return this->template logLikelihoodImpl<dual2nd>(xd);
        };
        H = hessian(func, wrt(xdual), at(xdual), fdual, g);
        return static_cast<double>(fdual);
    }

    void MeasurementFieldLandmarks::update(SystemBase& system_) {
        SystemEstimator& system          = dynamic_cast<SystemEstimator&>(system_);
        const GaussianInfo<double> prior = system.density;

        // Iterated re-association (cf. iterative landmark matching): optimise with
        // the current association, then re-associate at the posterior mean; if the
        // association set changed, restore the prior and re-run.
        for (int iteration = 0; iteration < maxAssociationIterations_; ++iteration) {
            if (uMeas_.cols() == 0) {
                system.density = prior;  // Nothing associated; leave the prior untouched
                return;
            }

            Measurement::update(system);

            if (iteration == maxAssociationIterations_ - 1) {
                break;  // Iteration budget exhausted
            }

            std::vector<std::pair<std::size_t, std::size_t>> newKeys =
                associate(system.density.mean(), system.density.cov());
            if (newKeys == assocKeys_) {
                break;  // Association converged (associate() rebuilt the same set)
            }
            assocKeys_     = newKeys;
            system.density = prior;
        }
    }
}  // namespace utility::slam::measurement
