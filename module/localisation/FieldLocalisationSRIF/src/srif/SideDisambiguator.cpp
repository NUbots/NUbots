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
#include "SideDisambiguator.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <vector>

#include "utility/gaussian_filtering/rotation.hpp"
#include "utility/vision/projection.hpp"

namespace module::localisation::srif {

    namespace {
        /// @brief True if a pixel lies within the image bounds.
        bool in_image(const Eigen::Vector2d& px, const Eigen::Vector2d& dimensions) {
            return px.x() >= 0.0 && px.x() < dimensions.x() && px.y() >= 0.0 && px.y() < dimensions.y();
        }
    }  // namespace

    SideDisambiguator::SideDisambiguator(const message::input::Image::Lens& lens,
                                         const Eigen::Vector2d& dimensions,
                                         const FieldDimensions& dims,
                                         const Options& opts)
        : options(opts)
        , lens_(lens)
        , dimensions_(dimensions)
        , detector_(lens, dimensions, dims)
        , half_carpet_length_(dims.field_length / 2 + dims.border_strip_min_width + opts.field_margin)
        , half_carpet_width_(dims.field_width / 2 + dims.border_strip_min_width + opts.field_margin) {}

    SideDisambiguator::SideDisambiguator(const message::input::Image::Lens& lens,
                                         const Eigen::Vector2d& dimensions,
                                         const FieldDimensions& dims)
        : SideDisambiguator(lens, dimensions, dims, Options{}) {}

    bool SideDisambiguator::is_background_point(const Eigen::Vector3d& rPFf) const {
        // Plausible static background: beyond the carpet in plan view, or overhead
        // structure (ceiling lights/trusses above the field are static and useful).
        // Reject underground or absurdly high solutions outright.
        if (rPFf.z() < -1.0 || rPFf.z() > 15.0) {
            return false;
        }
        const bool off_carpet = std::abs(rPFf.x()) > half_carpet_length_ || std::abs(rPFf.y()) > half_carpet_width_;
        return off_carpet || rPFf.z() > options.min_height_on_carpet;
    }

    SideDisambiguator::TriResult SideDisambiguator::triangulate(const std::vector<Landmark::Obs>& obs,
                                                                Eigen::Vector3d& rPFf,
                                                                Eigen::Matrix3d& P,
                                                                double& mean_chi2) const {
        if (obs.size() < 2) {
            return TriResult::GEOMETRY;
        }

        // Linear least squares for the point minimising the perpendicular distance
        // to every observation ray: sum_i || (I - u_i u_i^T)(p - c_i) ||^2.
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (const Landmark::Obs& o : obs) {
            const Eigen::Matrix3d M = Eigen::Matrix3d::Identity() - o.uFf * o.uFf.transpose();
            A += M;
            b += M * o.rCFf;
        }

        // Depth is unobservable without parallax: the normal matrix is singular
        // along the (common) ray direction.
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(A);
        if (es.info() != Eigen::Success || es.eigenvalues()(0) < 1e-3) {
            return TriResult::GEOMETRY;
        }
        rPFf = A.ldlt().solve(b);

        // Static-consistency test: every observation must point at the solved point
        // to within the angular residual scale. Dynamic objects (crowd, robots)
        // cannot satisfy this once there is parallax in the window.
        const double sigma2  = options.sigma_static * options.sigma_static;
        double chi2          = 0.0;
        Eigen::Matrix3d info = Eigen::Matrix3d::Identity() * (1.0 / (options.max_range * options.max_range));
        for (const Landmark::Obs& o : obs) {
            const Eigen::Vector3d rel = rPFf - o.rCFf;
            const double range        = rel.norm();
            const double depth        = rel.dot(o.uFf);
            if (depth < options.min_range || range > options.max_range) {
                return TriResult::RANGE;  // Behind or implausibly close/far for a background point
            }
            const Eigen::Matrix3d M = Eigen::Matrix3d::Identity() - o.uFf * o.uFf.transpose();
            chi2 += (M * rel).squaredNorm() / (sigma2 * range * range);
            info += M / (sigma2 * range * range);
        }
        mean_chi2 = chi2 / obs.size();
        if (mean_chi2 > options.static_chi2_mean) {
            return TriResult::CHI2;
        }
        P = info.inverse();
        return TriResult::OK;
    }

    bool SideDisambiguator::fit_far(const std::vector<Landmark::Obs>& obs,
                                    Eigen::Vector3d& rPFf,
                                    Eigen::Matrix3d& P) const {
        Eigen::Vector3d u_sum  = Eigen::Vector3d::Zero();
        Eigen::Vector3d c_mean = Eigen::Vector3d::Zero();
        for (const Landmark::Obs& o : obs) {
            u_sum += o.uFf;
            c_mean += o.rCFf;
        }
        if (u_sum.norm() < 1e-9) {
            return false;
        }
        const Eigen::Vector3d u_mean = u_sum.normalized();
        c_mean /= obs.size();

        double spread2 = 0.0;
        for (const Landmark::Obs& o : obs) {
            const double a = std::acos(std::clamp(o.uFf.dot(u_mean), -1.0, 1.0));
            spread2 += a * a;
        }
        const double spread_rms = std::sqrt(spread2 / obs.size());
        if (spread_rms > options.far_max_spread) {
            return false;  // Jittery bearings: dynamic object or track hopping
        }

        rPFf                    = c_mean + options.assumed_range * u_mean;
        const double sigma_r    = 0.5 * options.assumed_range;  // Depth stand-in
        const double sigma_perp = std::max(spread_rms, options.sigma_static) * options.assumed_range;
        P                       = sigma_r * sigma_r * u_mean * u_mean.transpose()
            + sigma_perp * sigma_perp * (Eigen::Matrix3d::Identity() - u_mean * u_mean.transpose());
        return true;
    }

    std::vector<SideDisambiguator::Association> SideDisambiguator::associate(
        const std::vector<OutOfFieldFeature>& features,
        const Pose<double>& Tfc,
        double pos_std,
        double yaw_std,
        double& score,
        std::vector<Prediction>& predictions,
        std::vector<char>& feature_outlier) const {
        score = 0.0;
        predictions.clear();
        feature_outlier.assign(features.size(), 0);

        const Eigen::Matrix3d Rcf  = Tfc.rotation_matrix.transpose();
        const Eigen::Vector3d rCFf = Tfc.translation_vector;
        const double log_clutter   = std::log(options.clutter_density);
        const double cos_pre_gate  = std::cos(options.pre_gate_angle);
        const double eff_pos_std   = std::max(pos_std, options.pos_std_floor);
        const Eigen::Matrix3d Pcam = Eigen::Matrix3d::Identity() * eff_pos_std * eff_pos_std;

        // Predict every landmark into this camera and precompute its predictive
        // density in the ray tangent plane. Predictions that project into the image
        // are all reported (the visualiser draws them); the ambiguous ones simply
        // take no part in matching or scoring.
        struct Predicted {
            Eigen::Vector3d uFf;            ///< Predicted unit ray in {f}
            Eigen::Matrix<double, 3, 2> T;  ///< Tangent basis at the predicted ray
            Eigen::Matrix2d Sinv;           ///< Inverse innovation covariance
            double half_log_det_2pi_S;      ///< 0.5*log det(2 pi S)
        };
        std::vector<Predicted> predicted;  ///< Parallel to predictions
        predicted.reserve(landmarks_.size());
        predictions.reserve(landmarks_.size());
        for (std::size_t j = 0; j < landmarks_.size(); ++j) {
            const Landmark& lm        = landmarks_[j];
            const Eigen::Vector3d rel = lm.rPFf - rCFf;
            const double range        = rel.norm();
            if (range < options.min_range)
                continue;

            const Eigen::Vector3d uFf = rel / range;
            const Eigen::Vector3d uCc = Rcf * uFf;
            // Only rays in the camera's forward hemisphere can land on the sensor; a ray at or behind
            // the horizon projects to a radius that says nothing about where it would be seen.
            if (uCc.x() <= 1e-3)
                continue;
            const Eigen::Vector2d px = utility::vision::project_pixel(uCc, lens_, dimensions_);
            if (!in_image(px, dimensions_))
                continue;

            Predicted pr;
            pr.uFf = uFf;
            pr.T   = tangent_basis(uFf);

            // Innovation covariance in the tangent plane: bearing noise + landmark
            // and camera position uncertainty projected across the range + camera
            // yaw uncertainty (rotation about field-up).
            const Eigen::Vector2d a = pr.T.transpose() * Eigen::Vector3d::UnitZ().cross(uFf);
            Eigen::Matrix2d S       = Eigen::Matrix2d::Identity() * (options.sigma_angular * options.sigma_angular)
                                + pr.T.transpose() * (lm.P + Pcam) * pr.T / (range * range)
                                + yaw_std * yaw_std * a * a.transpose();

            // A bearing-only landmark viewed far from its anchor smears into a long
            // thin acceptance corridor (its radial depth variance leaks into the
            // tangent plane). Such a prediction cannot discriminate anything at
            // this baseline: exclude the landmark from matching rather than let
            // corridor matches alias, and don't count it as predicted-visible either.
            const double tr_S = S.trace();
            const double dS   = std::sqrt(std::max(0.25 * tr_S * tr_S - S.determinant(), 0.0));

            Prediction out;
            out.landmark   = j;
            out.px         = px;
            const double m = options.visible_margin;
            out.well_inside =
                px.x() >= m && px.x() < dimensions_.x() - m && px.y() >= m && px.y() < dimensions_.y() - m;
            out.ambiguous = 0.5 * tr_S + dS > options.max_tangent_sigma * options.max_tangent_sigma;

            pr.Sinv               = S.inverse();
            pr.half_log_det_2pi_S = 0.5 * std::log(S.determinant()) + std::log(2.0 * M_PI);
            predicted.push_back(pr);
            predictions.push_back(out);
        }

        // Surprisal of every gated (feature, landmark) pair. The clutter hypothesis
        // (uniform density over direction space) sets the acceptance threshold: an
        // association is only worth making if the predictive density at the residual
        // beats the clutter density.
        struct Pair {
            double surprisal;
            std::size_t f;
            std::size_t p;
        };
        std::vector<Pair> pairs;
        for (std::size_t i = 0; i < features.size(); ++i) {
            if (!features[i].out_of_field)
                continue;
            const Eigen::Vector3d u_meas_f = Tfc.rotation_matrix * features[i].uPCc;
            for (std::size_t k = 0; k < predicted.size(); ++k) {
                if (predictions[k].ambiguous)
                    continue;
                const Predicted& pr = predicted[k];
                if (u_meas_f.dot(pr.uFf) < cos_pre_gate)
                    continue;
                const int dist = static_cast<int>(
                    cv::norm(landmarks_[predictions[k].landmark].descriptor, features[i].descriptor, cv::NORM_HAMMING));
                if (dist > options.max_descriptor_distance)
                    continue;
                // Past both gates this corner is a plausible sighting of this
                // landmark; if it still ends the frame unassociated it was rejected
                // on the evidence, which is worth telling the visualiser apart from
                // a corner nothing ever proposed a match for.
                feature_outlier[i]      = 1;
                const Eigen::Vector2d e = pr.T.transpose() * (u_meas_f - pr.uFf);
                const double s          = 0.5 * e.dot(pr.Sinv * e) + pr.half_log_det_2pi_S;
                if (s < -log_clutter) {
                    pairs.push_back({s, i, k});
                }
            }
        }

        // Greedy one-to-one assignment by ascending surprisal (SNN).
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.surprisal < b.surprisal; });
        std::vector<bool> feat_taken(features.size(), false);
        std::vector<Association> assoc;
        for (const Pair& pq : pairs) {
            if (feat_taken[pq.f] || predictions[pq.p].associated)
                continue;
            feat_taken[pq.f]             = true;
            feature_outlier[pq.f]        = 0;
            predictions[pq.p].associated = true;
            predictions[pq.p].feature    = pq.f;
            assoc.push_back({pq.f, predictions[pq.p].landmark, pq.surprisal});
            // Robust evidence: log ratio of the inlier predictive density to the
            // clutter density (positive by the acceptance gate above).
            score += -pq.surprisal - log_clutter;
        }
        return assoc;
    }

    void SideDisambiguator::update_candidates(const std::vector<OutOfFieldFeature>& features,
                                              const std::vector<bool>& feature_used,
                                              const Pose<double>& Tfc,
                                              double t,
                                              std::vector<char>& feature_grew_track) {
        feature_grew_track.assign(features.size(), 0);
        const Eigen::Vector3d rCFf = Tfc.translation_vector;
        const double cos_gate      = std::cos(options.cand_gate_angle);

        // Gated (feature, candidate) pairs ranked by descriptor distance, matched
        // greedily one-to-one. The geometric gate compares against the candidate's
        // most recent bearing (no depth yet, so a generous angular gate stands in
        // for a proper prediction).
        struct Pair {
            int dist;
            std::size_t f;
            std::size_t c;
        };
        std::vector<Pair> pairs;
        for (std::size_t i = 0; i < features.size(); ++i) {
            if (!features[i].out_of_field || feature_used[i])
                continue;
            const Eigen::Vector3d u_meas_f = Tfc.rotation_matrix * features[i].uPCc;
            for (std::size_t c = 0; c < candidates_.size(); ++c) {
                if (u_meas_f.dot(candidates_[c].obs.back().uFf) < cos_gate)
                    continue;
                const int dist =
                    static_cast<int>(cv::norm(candidates_[c].descriptor, features[i].descriptor, cv::NORM_HAMMING));
                if (dist > options.cand_max_descriptor_distance)
                    continue;
                pairs.push_back({dist, i, c});
            }
        }
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.dist < b.dist; });

        std::vector<bool> feat_matched(features.size(), false);
        std::vector<bool> cand_matched(candidates_.size(), false);
        for (const Pair& pq : pairs) {
            if (feat_matched[pq.f] || cand_matched[pq.c])
                continue;
            feat_matched[pq.f]       = true;
            cand_matched[pq.c]       = true;
            feature_grew_track[pq.f] = 1;

            Candidate& cand                = candidates_[pq.c];
            const Eigen::Vector3d u_meas_f = Tfc.rotation_matrix * features[pq.f].uPCc;
            if (cand.obs.size() >= static_cast<std::size_t>(options.obs_window)) {
                // Keep the oldest observation as the parallax/timespan anchor and
                // roll the rest of the window.
                cand.obs.erase(cand.obs.begin() + 1);
            }
            cand.obs.push_back({rCFf, u_meas_f, t});
            cand.descriptor = features[pq.f].descriptor.clone();
            cand.last_seen  = t;
        }

        // Promote mature candidates: enough observations over enough time with
        // enough parallax to triangulate, and consistent with one static point.
        std::vector<bool> cand_drop(candidates_.size(), false);
        for (std::size_t c = 0; c < candidates_.size(); ++c) {
            Candidate& cand = candidates_[c];
            if (!cand_matched[c]) {
                cand_drop[c] = t - cand.last_seen > options.cand_max_age;
                continue;
            }
            if (cand.obs.size() < static_cast<std::size_t>(options.min_obs))
                continue;
            if (cand.obs.back().t - cand.obs.front().t < options.min_time_span)
                continue;
            stats_.promote_attempts++;

            double max_parallax = 0.0;
            for (std::size_t a = 0; a < cand.obs.size(); ++a)
                for (std::size_t b2 = a + 1; b2 < cand.obs.size(); ++b2)
                    max_parallax =
                        std::max(max_parallax, std::acos(std::clamp(cand.obs[a].uFf.dot(cand.obs[b2].uFf), -1.0, 1.0)));
            if (max_parallax < options.min_parallax) {
                // No usable depth. If the track is long, old and directionally
                // tight, promote it as a bearing-only landmark: bearing alone
                // discriminates the mirror. Jittery tracks are dropped as dynamic.
                if (cand.obs.size() >= static_cast<std::size_t>(options.far_promote_obs)
                    && cand.obs.back().t - cand.obs.front().t >= options.far_promote_time_span) {
                    Eigen::Vector3d rPFf;
                    Eigen::Matrix3d P;
                    if (fit_far(cand.obs, rPFf, P) && is_background_point(rPFf)) {
                        stats_.promoted_far++;
                        Landmark lm;
                        lm.rPFf       = rPFf;
                        lm.P          = P;
                        lm.far        = true;
                        lm.descriptor = cand.descriptor.clone();
                        lm.hits       = static_cast<int>(cand.obs.size());
                        lm.last_seen  = t;
                        lm.obs        = cand.obs;
                        landmarks_.push_back(std::move(lm));
                    }
                    else {
                        stats_.far_spread_fail++;
                    }
                    cand_drop[c] = true;
                }
                else {
                    stats_.parallax_wait++;
                }
                continue;  // Keep waiting for baseline
            }

            Eigen::Vector3d rPFf;
            Eigen::Matrix3d P;
            double mean_chi2    = 0.0;
            const TriResult tri = triangulate(cand.obs, rPFf, P, mean_chi2);
            if (tri != TriResult::OK) {
                if (tri == TriResult::GEOMETRY)
                    stats_.tri_fail_geometry++;
                else if (tri == TriResult::RANGE)
                    stats_.tri_fail_range++;
                else
                    stats_.tri_fail_chi2++;
            }
            else if (!is_background_point(rPFf)) {
                stats_.background_fail++;
            }
            else {
                stats_.promoted++;
                Landmark lm;
                lm.rPFf       = rPFf;
                lm.P          = P;
                lm.descriptor = cand.descriptor.clone();
                lm.hits       = static_cast<int>(cand.obs.size());
                lm.last_seen  = t;
                lm.obs        = cand.obs;
                landmarks_.push_back(std::move(lm));
            }
            // Either way the candidate is finished: promoted, or inconsistent with a
            // static background point despite sufficient parallax (dynamic object).
            cand_drop[c] = true;
        }

        // Apply drops, then spawn new candidates from the remaining unmatched features.
        std::size_t w = 0;
        for (std::size_t c = 0; c < candidates_.size(); ++c) {
            if (cand_drop[c])
                continue;
            if (w != c)
                candidates_[w] = std::move(candidates_[c]);  // Guard the self-move
            ++w;
        }
        candidates_.resize(w);

        for (std::size_t i = 0; i < features.size(); ++i) {
            if (!features[i].out_of_field || feature_used[i] || feat_matched[i])
                continue;
            const Eigen::Vector3d uFf = Tfc.rotation_matrix * features[i].uPCc;
            // Overhead lighting grids are near-symmetric under the field's 180 deg
            // rotation; don't let high-elevation features into the map at all.
            if (std::asin(std::clamp(uFf.z(), -1.0, 1.0)) > options.max_elevation)
                continue;
            Candidate cand;
            cand.descriptor = features[i].descriptor.clone();
            cand.obs.push_back({rCFf, uFf, t});
            cand.last_seen = t;
            candidates_.push_back(std::move(cand));
        }

        // Cap the candidate pool. Established tracks (more observations) outrank
        // fresh single-observation spawns, so the cap churns the spawn pool rather
        // than evicting tracks that are accumulating parallax.
        if (candidates_.size() > options.max_candidates) {
            std::sort(candidates_.begin(), candidates_.end(), [](const Candidate& a, const Candidate& b) {
                if (a.obs.size() != b.obs.size())
                    return a.obs.size() > b.obs.size();
                return a.last_seen > b.last_seen;
            });
            candidates_.resize(options.max_candidates);
        }
    }

    SideDisambiguator::FrameResult SideDisambiguator::process(double t,
                                                              const cv::Mat& gray,
                                                              const Pose<double>& Tfc,
                                                              const Pose<double>& TfcMirror,
                                                              double pos_std,
                                                              double yaw_std,
                                                              double yaw_rate_abs,
                                                              double heading) {
        FrameResult res;

        res.features                                   = detector_.detect(gray, Tfc);
        const std::vector<OutOfFieldFeature>& features = res.features;
        res.n_features                                 = features.size();
        res.feature_status.assign(features.size(), FEATURE_ON_CARPET);
        for (std::size_t i = 0; i < features.size(); ++i) {
            if (features[i].out_of_field) {
                res.feature_status[i] = FEATURE_UNMATCHED;
                res.n_out_of_field++;
            }
        }

        // Associate the same corners against the map under both side hypotheses.
        std::vector<Prediction> pred_own, pred_mirror;
        std::vector<char> outlier_own, outlier_mirror;
        std::vector<Association> assoc_own =
            associate(features, Tfc, pos_std, yaw_std, res.score_own, pred_own, outlier_own);
        std::vector<Association> assoc_mirror =
            associate(features, TfcMirror, pos_std, yaw_std, res.score_mirror, pred_mirror, outlier_mirror);

        // Landmarks predicted comfortably inside the image (and able to discriminate)
        // are what makes a frame worth scoring; the unassociated ones feed the
        // miss-streak pruning below.
        std::vector<std::size_t> miss_own;
        std::size_t vis_own = 0, vis_mirror = 0;
        for (const Prediction& p : pred_own) {
            if (p.ambiguous || !p.well_inside)
                continue;
            vis_own++;
            if (!p.associated)
                miss_own.push_back(p.landmark);
        }
        for (const Prediction& p : pred_mirror) {
            if (!p.ambiguous && p.well_inside)
                vis_mirror++;
        }
        res.n_associated        = assoc_own.size();
        res.n_associated_mirror = assoc_mirror.size();
        res.n_visible_own       = vis_own;
        res.n_visible_mirror    = vis_mirror;

        // Per-corner status for the visualiser, in order of increasing precedence:
        // a corner rejected under one pose but associated under the other reads as
        // associated. Mirror-only matches are exactly the wrong-side evidence the
        // LLR accumulates, so they get their own colour rather than hiding.
        for (std::size_t i = 0; i < features.size(); ++i) {
            if (features[i].out_of_field && (outlier_own[i] || outlier_mirror[i])) {
                res.feature_status[i] = FEATURE_OUTLIER;
            }
        }
        for (const Association& a : assoc_mirror) {
            res.feature_status[a.feature] = FEATURE_MIRROR;
        }
        for (const Association& a : assoc_own) {
            res.feature_status[a.feature] = FEATURE_ASSOCIATED;
        }
        res.n_outlier = static_cast<std::size_t>(
            std::count(res.feature_status.begin(), res.feature_status.end(), static_cast<int>(FEATURE_OUTLIER)));

        // Projected map landmarks for the visualiser. Statuses are as they stand
        // after association; the maintenance pass below upgrades any that it culls.
        // The status is also stamped on the landmark itself so the 3D view can
        // colour the whole map: stored there it survives the compaction below,
        // which a landmarks_-indexed side table would not.
        for (Landmark& lm : landmarks_) {
            lm.last_status = LANDMARK_NOT_IN_VIEW;
        }
        std::vector<int> view_of_landmark(landmarks_.size(), -1);
        res.landmark_views.reserve(pred_own.size());
        for (const Prediction& p : pred_own) {
            LandmarkView lv;
            lv.px  = p.px;
            lv.far = landmarks_[p.landmark].far;
            if (p.associated) {
                lv.status   = LANDMARK_ASSOCIATED;
                lv.match_px = features[p.feature].px;
            }
            else if (p.ambiguous)
                lv.status = LANDMARK_AMBIGUOUS;
            else if (p.well_inside)
                lv.status = LANDMARK_MISSED;
            else
                lv.status = LANDMARK_EDGE;
            landmarks_[p.landmark].last_status = lv.status;
            view_of_landmark[p.landmark]       = static_cast<int>(res.landmark_views.size());
            res.landmark_views.push_back(lv);
        }

        // Accumulate the side evidence whenever the map could have discriminated
        // (some landmark was predicted visible under either hypothesis) and the
        // view is trustworthy (not mid-turn: motion blur and a freshly-panned
        // viewpoint starve the true side of matches without saying anything about
        // which side is right).
        const bool scored_frame = vis_own + vis_mirror > 0 && yaw_rate_abs < options.max_yaw_rate;
        if (scored_frame) {
            const double delta =
                std::clamp(res.score_own - res.score_mirror, -options.delta_clamp, options.delta_clamp);
            res.side_delta = delta;
            llr_           = std::clamp(options.forgetting * llr_ + delta, -options.llr_clamp, options.llr_clamp);
        }
        res.llr = llr_;

        // Deep doubt latches: only positive evidence (not forgetting-driven decay
        // towards zero) may re-arm map building on this side.
        if (llr_ <= -options.flip_threshold)
            doubt_ = true;
        else if (llr_ >= options.rebuild_llr)
            doubt_ = false;

        // A flip needs sustained, substantive and DOMINANT mirror evidence: the LLR
        // must sit below the threshold, each streak frame must carry enough
        // mirror-side associations that a couple of matches to symmetric structure
        // cannot flip a barely-covered map, and the mirror must clearly outnumber
        // the own side (a degraded own pose is "no decision", not mirror evidence).
        // Unscored frames leave the streak untouched.
        if (scored_frame) {
            if (llr_ <= -options.flip_threshold && res.n_associated_mirror >= options.min_flip_assoc
                && static_cast<double>(res.n_associated_mirror)
                       >= options.flip_dominance * static_cast<double>(res.n_associated)
                && static_cast<double>(vis_own) >= options.flip_coverage * static_cast<double>(vis_mirror)) {
                flip_streak_++;
            }
            else if (llr_ > -options.flip_threshold) {
                flip_streak_ = 0;  // Side no longer in doubt: clear the streak
            }
            else if (flip_streak_ > 0) {
                flip_streak_--;  // Still in doubt, frame unqualifying: leak, don't reset
            }

            // Remember where the robot was pointing the last time the map actually
            // confirmed the own side. That is the reference the turn gate below
            // measures against, so it deliberately needs a healthy match rather than
            // any match at all: during a turn the association count decays through
            // small non-zero values, and taking the last of those as the reference
            // would measure the turn from halfway through it.
            const bool own_confirmed =
                vis_own > 0
                && res.n_associated >= std::max(options.blind_match_min_assoc,
                                                static_cast<std::size_t>(std::ceil(options.blind_match_fraction
                                                                                   * static_cast<double>(vis_own))));
            if (own_confirmed) {
                heading_at_own_match_ = heading;
                have_own_match_       = true;
            }

            // Net heading change since then. A flip asserts a 180 deg discontinuity;
            // if the robot has already turned by about that much under its own
            // gyroscope, the mirror-looking view is what the turn predicts and the
            // out-of-field evidence cannot separate the two (see Options).
            const double turn    = have_own_match_ ? std::remainder(heading - heading_at_own_match_, 2.0 * M_PI) : 0.0;
            res.turn_since_match = turn;
            const bool turn_explains_mirror =
                have_own_match_ && std::abs(std::abs(turn) - M_PI) < options.blind_turn_tolerance;

            // Blind-own escape (see Options): near-clamp LLR, own essentially
            // blind, mirror matching real structure. Same leak/reset semantics.
            if (llr_ <= -options.flip_blind_llr && vis_own >= options.flip_blind_min_visible_own
                && res.n_associated <= options.flip_blind_own_max
                && res.n_associated_mirror
                       >= std::max(options.flip_blind_min_assoc,
                                   static_cast<std::size_t>(
                                       std::ceil(options.flip_dominance * static_cast<double>(res.n_associated))))
                && !turn_explains_mirror) {
                blind_streak_++;
            }
            else if (llr_ <= -options.flip_blind_llr
                     && (turn_explains_mirror || vis_own < options.flip_blind_min_visible_own)) {
                // Refused because the comparison is not a comparison: hold the streak
                // rather than leaking it, so a robot that turns back to mapped
                // territory neither flips nor re-earns the evidence from scratch.
                res.blind_turn_blocked = true;
            }
            else if (llr_ > -options.flip_threshold) {
                blind_streak_ = 0;
            }
            else if (blind_streak_ > 0) {
                blind_streak_--;
            }

            res.flip_requested =
                flip_streak_ >= options.flip_consecutive || blind_streak_ >= options.flip_blind_consecutive;
        }

        // Map maintenance. Frozen while the pose is uncertain, the side is in
        // doubt, or a flip just happened, so a wrong-side excursion cannot poison
        // the map.
        const bool frozen = pos_std > options.max_pos_std || llr_ < options.freeze_llr || doubt_ || res.flip_requested
                            || t < map_freeze_until_;
        res.map_frozen = frozen;
        if (!frozen) {
            std::vector<bool> landmark_dead(landmarks_.size(), false);
            std::vector<bool> feature_used(features.size(), false);
            // Flag a culled landmark in its view, if it has one this frame.
            auto mark_culled = [&](std::size_t idx, int status) {
                if (view_of_landmark[idx] >= 0)
                    res.landmark_views[view_of_landmark[idx]].status = status;
            };

            // Hits: extend the observation window and re-triangulate. A landmark
            // that stops fitting a static point (someone who stood still and then
            // moved) fails the consistency test and is culled.
            for (const Association& a : assoc_own) {
                feature_used[a.feature] = true;
                Landmark& lm            = landmarks_[a.landmark];
                if (lm.obs.size() >= static_cast<std::size_t>(options.obs_window)) {
                    // Keep the oldest observation as the parallax anchor.
                    lm.obs.erase(lm.obs.begin() + 1);
                }
                lm.obs.push_back({Tfc.translation_vector, Tfc.rotation_matrix * features[a.feature].uPCc, t});

                Eigen::Vector3d rPFf;
                Eigen::Matrix3d P;
                double mean_chi2    = 0.0;
                const TriResult tri = triangulate(lm.obs, rPFf, P, mean_chi2);
                if (tri == TriResult::OK && is_background_point(rPFf)) {
                    // Full depth solution available (possibly upgrading a
                    // bearing-only landmark that finally accrued parallax).
                    if (lm.far)
                        stats_.upgraded++;
                    lm.far        = false;
                    lm.rPFf       = rPFf;
                    lm.P          = P;
                    lm.descriptor = features[a.feature].descriptor.clone();
                    lm.hits++;
                    lm.miss_streak = 0;
                    lm.last_seen   = t;
                }
                else if (lm.far && tri != TriResult::CHI2) {
                    // Bearing-only landmark still without parallax: refit the
                    // bearing; a jittery window means a mover, so cull it.
                    if (fit_far(lm.obs, rPFf, P) && is_background_point(rPFf)) {
                        lm.rPFf       = rPFf;
                        lm.P          = P;
                        lm.descriptor = features[a.feature].descriptor.clone();
                        lm.hits++;
                        lm.miss_streak = 0;
                        lm.last_seen   = t;
                    }
                    else {
                        landmark_dead[a.landmark] = true;
                        mark_culled(a.landmark, LANDMARK_CULLED_OUTLIER);
                        stats_.landmark_culled_chi2++;
                    }
                }
                else {
                    landmark_dead[a.landmark] = true;
                    mark_culled(a.landmark, LANDMARK_CULLED_OUTLIER);
                    stats_.landmark_culled_chi2++;
                }
            }

            // Misses: predicted comfortably inside the image but not re-observed.
            for (std::size_t idx : miss_own) {
                if (++landmarks_[idx].miss_streak > options.max_miss_streak) {
                    landmark_dead[idx] = true;
                    mark_culled(idx, LANDMARK_CULLED_MISSING);
                    stats_.landmark_culled_miss++;
                }
            }

            std::size_t w = 0;
            for (std::size_t j = 0; j < landmarks_.size(); ++j) {
                if (landmark_dead[j])
                    continue;
                if (w != j)
                    landmarks_[w] = std::move(landmarks_[j]);  // Guard the self-move
                ++w;
            }
            landmarks_.resize(w);

            std::vector<char> grew_track;
            update_candidates(features, feature_used, Tfc, t, grew_track);
            for (std::size_t i = 0; i < features.size(); ++i) {
                if (grew_track[i] && res.feature_status[i] < FEATURE_CANDIDATE) {
                    res.feature_status[i] = FEATURE_CANDIDATE;
                }
            }

            // Cap the map, keeping the landmarks that are earning their place:
            // re-observations discounted by how long ago they stopped arriving.
            if (landmarks_.size() > options.max_landmarks) {
                const double half_life = std::max(options.evict_half_life, 1e-3);
                auto cap_score         = [&](const Landmark& lm) {
                    return lm.hits / (1.0 + std::max(0.0, t - lm.last_seen) / half_life);
                };
                std::sort(landmarks_.begin(), landmarks_.end(), [&](const Landmark& a, const Landmark& b) {
                    return cap_score(a) > cap_score(b);
                });
                landmarks_.resize(options.max_landmarks);
            }
        }

        res.n_landmarks  = landmarks_.size();
        res.n_candidates = candidates_.size();
        return res;
    }

    void SideDisambiguator::notify_flip_applied(double t) {
        llr_              = -llr_;
        flip_streak_      = 0;
        blind_streak_     = 0;
        map_freeze_until_ = t + options.flip_cooldown;
    }

}  // namespace module::localisation::srif
