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
#include "OutOfFieldFeatures.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace module::localisation::srif {

    OutOfFieldDetector::OutOfFieldDetector(const message::input::Image::Lens& lens,
                                           const Eigen::Vector2d& dimensions,
                                           const FieldDimensions& dims,
                                           const Options& opts)
        : options(opts)
        , lens_(lens)
        , dimensions_(dimensions)
        , half_carpet_length_(dims.field_length / 2 + dims.border_strip_min_width + opts.field_margin)
        , half_carpet_width_(dims.field_width / 2 + dims.border_strip_min_width + opts.field_margin)
        , orb_(cv::ORB::create()) {}

    OutOfFieldDetector::OutOfFieldDetector(const message::input::Image::Lens& lens,
                                           const Eigen::Vector2d& dimensions,
                                           const FieldDimensions& dims)
        : OutOfFieldDetector(lens, dimensions, dims, Options{}) {}

    bool OutOfFieldDetector::is_out_of_field(const Eigen::Vector3d& uPCc, const Pose<double>& Tfc) const {
        // Ray direction in the field frame and camera position above the ground plane
        const Eigen::Vector3d uFf  = Tfc.rotation_matrix * uPCc;
        const Eigen::Vector3d rCFf = Tfc.translation_vector;

        // A ray at or above the horizon never reaches the carpet
        if (uFf.z() >= -options.horizon_margin_z) {
            return true;
        }

        // Intersect the ground plane z = 0 and test against the carpet extent
        const double t          = -rCFf.z() / uFf.z();
        const Eigen::Vector3d g = rCFf + t * uFf;
        return std::abs(g.x()) > half_carpet_length_ || std::abs(g.y()) > half_carpet_width_;
    }

    std::vector<OutOfFieldFeature> OutOfFieldDetector::detect(const cv::Mat& gray, const Pose<double>& Tfc) const {
        assert(gray.type() == CV_8UC1);

        std::vector<cv::KeyPoint> keypoints;
        cv::FAST(gray, keypoints, options.fast_threshold, /*nonmaxSuppression*/ true);

        // Reject corners near the image edge, then keep the strongest max_features
        const double b = options.image_border;
        std::erase_if(keypoints, [&](const cv::KeyPoint& kp) {
            return kp.pt.x < b || kp.pt.x >= gray.cols - b || kp.pt.y < b || kp.pt.y >= gray.rows - b;
        });
        std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& c) {
            return a.response > c.response;
        });
        if (keypoints.size() > static_cast<std::size_t>(options.max_features)) {
            keypoints.resize(options.max_features);
        }

        // ORB::compute may drop keypoints, and keeps `descriptors` rows aligned with `keypoints`
        cv::Mat descriptors;
        orb_->compute(gray, keypoints, descriptors);

        std::vector<OutOfFieldFeature> features;
        features.reserve(keypoints.size());
        for (std::size_t i = 0; i < keypoints.size(); ++i) {
            OutOfFieldFeature f;
            f.px           = Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y);
            f.uPCc         = utility::vision::unproject_pixel(f.px, lens_, dimensions_);
            f.descriptor   = descriptors.row(static_cast<int>(i));
            f.response     = keypoints[i].response;
            f.out_of_field = is_out_of_field(f.uPCc, Tfc);
            features.push_back(std::move(f));
        }
        return features;
    }
}  // namespace module::localisation::srif
