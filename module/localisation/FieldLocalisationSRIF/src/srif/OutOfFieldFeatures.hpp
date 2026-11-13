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
 * @file OutOfFieldFeatures.h
 * @brief FAST corner detection and out-of-field classification for side disambiguation.
 *
 * Extracts features from a camera frame for SideDisambiguator:
 *  - FAST corners on the grayscale image (strongest max_features kept),
 *  - an ORB descriptor per corner,
 *  - a unit ray in the camera frame {c} per corner,
 *  - an out-of-field classification given the estimated camera pose in {f}: a ray is
 *    out-of-field when it points at or above the horizon, or meets the ground plane
 *    outside the field carpet (boundary + border strip + margin).
 *
 * The classification is the same under a pose and its 180 degree mirror.
 */
#ifndef MODULE_LOCALISATION_SRIF_OUTOFFIELDFEATURES_HPP
#define MODULE_LOCALISATION_SRIF_OUTOFFIELDFEATURES_HPP

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

#include "srif/FieldMap.hpp"

#include "message/input/Image.hpp"

#include "utility/gaussian_filtering/Pose.hpp"
#include "utility/vision/projection.hpp"

namespace module::localisation::srif {

    using utility::gaussian_filtering::Pose;

    /// @brief One detected corner feature, with its camera ray and classification
    struct OutOfFieldFeature {
        Eigen::Vector2d px;         ///< Pixel position (x right, y down)
        Eigen::Vector3d uPCc;       ///< Unit ray in the camera frame {c}
        cv::Mat descriptor;         ///< 1x32 CV_8U ORB descriptor (row view into the detection batch)
        float response    = 0.0f;   ///< FAST corner response
        bool out_of_field = false;  ///< True if the ray looks beyond the field carpet
    };

    /// @brief Detects FAST corners and classifies them as on-carpet or out-of-field
    class OutOfFieldDetector {
    public:
        /// @brief Detection and classification options
        struct Options {
            int fast_threshold      = 25;    ///< FAST intensity threshold (with non-max suppression)
            int max_features        = 300;   ///< Keep at most this many strongest corners
            int image_border        = 20;    ///< Reject corners within this many pixels of the image edge
            double field_margin     = 0.30;  ///< Extra margin beyond the border strip still counted as carpet [m]
            double horizon_margin_z = 0.02;  ///< Rays with field-frame z >= -margin count as at/above the horizon
        };

        /**
         * @brief Construct the detector.
         * @param lens Width-normalised lens calibration, as carried on message::input::Image
         * @param dimensions Image dimensions in pixels {width, height}
         * @param dims Field dimensions defining the carpet extent
         * @param options Detection and classification options
         */
        OutOfFieldDetector(const message::input::Image::Lens& lens,
                           const Eigen::Vector2d& dimensions,
                           const FieldDimensions& dims,
                           const Options& options);

        /// @brief Construct with default options.
        OutOfFieldDetector(const message::input::Image::Lens& lens,
                           const Eigen::Vector2d& dimensions,
                           const FieldDimensions& dims);

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
        bool is_out_of_field(const Eigen::Vector3d& uPCc, const Pose<double>& Tfc) const;

        Options options;

    private:
        message::input::Image::Lens lens_;  ///< Width-normalised lens calibration
        Eigen::Vector2d dimensions_;        ///< Image dimensions in pixels {width, height}
        double half_carpet_length_;         ///< Field half-length + border strip + margin [m]
        double half_carpet_width_;          ///< Field half-width + border strip + margin [m]
        cv::Ptr<cv::ORB> orb_;              ///< Descriptor extractor
    };
}  // namespace module::localisation::srif

#endif  // MODULE_LOCALISATION_SRIF_OUTOFFIELDFEATURES_HPP
