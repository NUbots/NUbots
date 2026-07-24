/**
 * @file OutOfFieldFeatures.h
 * @brief FAST corner detection and out-of-field classification for side disambiguation.
 *
 * The RoboCup field is symmetric under a 180 degree rotation about its centre, so
 * on-field landmarks cannot distinguish a pose from its mirror. The scenery beyond
 * the field boundary (walls, posters, furniture, spectators) is not symmetric, so
 * bearings to persistent out-of-field corner features carry exactly the information
 * the field landmarks lack.
 *
 * This module extracts those features from a camera frame:
 *  - FAST corners on the grayscale image (strongest maxFeatures kept),
 *  - an ORB descriptor per corner (oriented BRIEF, matchable across frames),
 *  - a unit ray in the camera frame {c} per corner (FisheyeLens::unproject),
 *  - an out-of-field classification given the estimated camera pose in {f}:
 *    a ray is out-of-field when it points at/above the horizon or its ground-plane
 *    intersection lands outside the field carpet (boundary + border strip + margin).
 *
 * The classification is invariant to the 180 degree field symmetry: the carpet
 * region of the image is the same under a pose and its mirror, so a wrong-side
 * estimate still masks the field correctly.
 */
#ifndef OUTOFFIELDFEATURES_HPP
#define OUTOFFIELDFEATURES_HPP

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

#include "FieldMap.hpp"
#include "camera/FisheyeLens.hpp"
#include "camera/Pose.hpp"

namespace utility::slam {

    using utility::slam::camera::FisheyeLens;
    using utility::slam::camera::Pose;

    /**
     * @brief One detected corner feature, with its camera ray and classification.
     */
    struct OutOfFieldFeature {
        Eigen::Vector2d px;       ///< Pixel position (x right, y down)
        Eigen::Vector3d uPCc;     ///< Unit ray in the camera frame {c}
        cv::Mat descriptor;       ///< 1x32 CV_8U ORB descriptor (row view into the detection batch)
        float response  = 0.0f;   ///< FAST corner response
        bool outOfField = false;  ///< True if the ray looks beyond the field carpet
    };

    /**
     * @brief Detects FAST corners and classifies them as on-carpet or out-of-field.
     */
    class OutOfFieldDetector {
    public:
        /**
         * @brief Detection and classification options.
         */
        struct Options {
            int fastThreshold     = 25;    ///< FAST intensity threshold (with non-max suppression)
            int maxFeatures       = 300;   ///< Keep at most this many strongest corners
            int imageBorder       = 20;    ///< Reject corners within this many pixels of the image edge
            double fieldMargin    = 0.30;  ///< Extra margin beyond the border strip still counted as carpet [m]
            double horizonMarginZ = 0.02;  ///< Rays with field-frame z >= -margin count as at/above the horizon
        };

        /**
         * @brief Construct the detector.
         * @param lens Fisheye lens model for pixel -> ray unprojection
         * @param dims Field dimensions defining the carpet extent
         * @param options Detection and classification options
         */
        OutOfFieldDetector(const FisheyeLens& lens, const FieldDimensions& dims, const Options& options);

        /// @brief Construct with default options.
        OutOfFieldDetector(const FisheyeLens& lens, const FieldDimensions& dims);

        /**
         * @brief Detect corners in a frame and classify them against the field extent.
         * @param gray Grayscale camera frame (CV_8UC1, full lens resolution)
         * @param Tfc Estimated camera pose in the field frame {f}
         * @return Detected features, strongest first, each with ray and classification
         */
        std::vector<OutOfFieldFeature> detect(const cv::Mat& gray, const Pose<double>& Tfc) const;

        /**
         * @brief Classify a single camera ray against the field extent.
         * @param uPCc Unit ray in {c}
         * @param Tfc Camera pose in {f}
         * @return True if the ray looks beyond the field carpet (or at/above the horizon)
         */
        bool isOutOfField(const Eigen::Vector3d& uPCc, const Pose<double>& Tfc) const;

        Options options;

    private:
        const FisheyeLens& lens_;
        double halfCarpetLength_;  ///< Field half-length + border strip + margin [m]
        double halfCarpetWidth_;   ///< Field half-width + border strip + margin [m]
        cv::Ptr<cv::ORB> orb_;     ///< Descriptor extractor (compute only; detection is FAST)
    };
}  // namespace utility::slam

#endif
