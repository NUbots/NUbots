/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#include "imagefeatures.hpp"

#include <algorithm>
#include <print>
#include <string>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace utility::slam::vision {

    cv::Mat detectAndDrawFAST(const cv::Mat& img, int maxNumFeatures) {
        cv::Mat imgout = img.clone();
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);  // FAST detector expects grayscale input.
        std::vector<cv::KeyPoint> keypoints;          // Store keypoints computed by FAST detector.
        cv::Ptr<cv::FastFeatureDetector> detector =
            cv::FastFeatureDetector::create(65, true, cv::FastFeatureDetector::TYPE_9_16);
        detector->detect(gray, keypoints);

        int suppressionRadius = 20;
        std::vector<cv::KeyPoint> suppressed_keypoints;
        for (const auto& candidate : keypoints) {
            bool suppress = false;
            for (const auto& accepted : suppressed_keypoints) {
                double distance = cv::norm(candidate.pt - accepted.pt);
                if (distance < suppressionRadius) {
                    suppress = true;
                    break;
                }
            }
            if (!suppress) {
                suppressed_keypoints.push_back(candidate);
            }
        }
        keypoints = suppressed_keypoints;

        // Sort by response score (descending).
        std::sort(keypoints.begin(), keypoints.end(), [](const auto& a, const auto& b) {
            return a.response > b.response;
        });

        // Draw all detected corners (now suppressed) in green
        for (const auto& kp : keypoints) {
            cv::circle(imgout, kp.pt, 3, cv::Scalar(0, 255, 0), 1);  // Green in BGR
        }

        std::println("Image width: {}", img.cols);
        std::println("Image height: {}", img.rows);
        std::println("Features requested: {}", maxNumFeatures);
        std::println("Features detected: {}", keypoints.size());
        std::println("{:<5} {:<10} {:<10} {:<10}", "Index", "X", "Y", "Score");

        // Limit to maxNumFeatures.
        if (keypoints.size() > static_cast<float>(maxNumFeatures)) {
            keypoints.resize(maxNumFeatures);
        }

        // Print each feature
        for (size_t i = 0; i < keypoints.size(); ++i) {
            const auto& kp = keypoints[i];
            std::println("{:<5} {:<10} {:<10} {:.8f}", i + 1, kp.pt.x, kp.pt.y, kp.response);
        }
        // Draw on output image
        for (size_t i = 0; i < keypoints.size(); ++i) {
            const auto& kp = keypoints[i];

            // Draw a red circle at the feature
            cv::circle(imgout, kp.pt, 4, cv::Scalar(0, 0, 255), 1);

            // Label the feature with its index number
            std::string label = std::to_string(i + 1);
            cv::putText(imgout,
                        label,
                        kp.pt + cv::Point2f(5, -5),  // offset text position
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.9,                    // font scale
                        cv::Scalar(255, 0, 0),  // green text
                        2);                     // thickness
        }

        return imgout;
    }

}  // namespace utility::slam::vision
