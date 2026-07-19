#include "OutOfFieldFeatures.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace utility::slam {

    OutOfFieldDetector::OutOfFieldDetector(const FisheyeLens& lens, const FieldDimensions& dims, const Options& opts)
        : options(opts)
        , lens_(lens)
        , halfCarpetLength_(dims.fieldLength / 2 + dims.borderStripMinWidth + opts.fieldMargin)
        , halfCarpetWidth_(dims.fieldWidth / 2 + dims.borderStripMinWidth + opts.fieldMargin)
        , orb_(cv::ORB::create()) {}

    OutOfFieldDetector::OutOfFieldDetector(const FisheyeLens& lens, const FieldDimensions& dims)
        : OutOfFieldDetector(lens, dims, Options{}) {}

    bool OutOfFieldDetector::isOutOfField(const Eigen::Vector3d& uPCc, const Pose<double>& Tfc) const {
        // Ray direction in the field frame and camera position above the ground plane.
        const Eigen::Vector3d uFf  = Tfc.rotationMatrix * uPCc;
        const Eigen::Vector3d rCFf = Tfc.translationVector;

        // At or above the horizon: the ray never reaches the carpet, so whatever it
        // sees is background scenery.
        if (uFf.z() >= -options.horizonMarginZ) {
            return true;
        }

        // Downward ray: intersect the ground plane z = 0 and test against the carpet
        // extent (field + border strip + margin).
        const double t          = -rCFf.z() / uFf.z();
        const Eigen::Vector3d g = rCFf + t * uFf;
        return std::abs(g.x()) > halfCarpetLength_ || std::abs(g.y()) > halfCarpetWidth_;
    }

    std::vector<OutOfFieldFeature> OutOfFieldDetector::detect(const cv::Mat& gray, const Pose<double>& Tfc) const {
        assert(gray.type() == CV_8UC1);

        std::vector<cv::KeyPoint> keypoints;
        cv::FAST(gray, keypoints, options.fastThreshold, /*nonmaxSuppression*/ true);

        // Reject corners near the image edge (unreliable unprojection and no room
        // for the descriptor patch), then keep the strongest maxFeatures.
        const double b = options.imageBorder;
        std::erase_if(keypoints, [&](const cv::KeyPoint& kp) {
            return kp.pt.x < b || kp.pt.x >= gray.cols - b || kp.pt.y < b || kp.pt.y >= gray.rows - b;
        });
        std::sort(keypoints.begin(), keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& c) {
            return a.response > c.response;
        });
        if (keypoints.size() > static_cast<std::size_t>(options.maxFeatures)) {
            keypoints.resize(options.maxFeatures);
        }

        // Oriented BRIEF descriptors at the surviving corners. ORB::compute may drop
        // keypoints (e.g. too close to its own border) and reorders nothing else, so
        // rows of `descriptors` stay aligned with `keypoints` afterwards.
        cv::Mat descriptors;
        orb_->compute(gray, keypoints, descriptors);

        std::vector<OutOfFieldFeature> features;
        features.reserve(keypoints.size());
        for (std::size_t i = 0; i < keypoints.size(); ++i) {
            OutOfFieldFeature f;
            f.px         = Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y);
            f.uPCc       = lens_.unproject(f.px);
            f.descriptor = descriptors.row(static_cast<int>(i));
            f.response   = keypoints[i].response;
            f.outOfField = isOutOfField(f.uPCc, Tfc);
            features.push_back(std::move(f));
        }
        return features;
    }
}  // namespace utility::slam
