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

#ifndef UTILITY_SLAM_VISION_IMAGE_FEATURES_HPP
#define UTILITY_SLAM_VISION_IMAGE_FEATURES_HPP

#include <vector>

#include <opencv2/core.hpp>

namespace utility::slam::vision {

    struct ArucoDetectionResult {
        cv::Mat image;
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
    };

    struct ShiTomasiDetectionResult {
        cv::Mat image;
        std::vector<cv::Point2f> points;
        std::vector<float> scores;
    };

    cv::Mat detectAndDrawHarris(const cv::Mat& img, int maxNumFeatures);
    ShiTomasiDetectionResult detectAndDrawShiAndTomasi(const cv::Mat& img, int maxNumFeatures);
    cv::Mat detectAndDrawFAST(const cv::Mat& img, int maxNumFeatures);
    ArucoDetectionResult detectAndDrawArUco(const cv::Mat& img, int maxNumFeatures);

}  // namespace utility::slam::vision

#endif  // UTILITY_SLAM_VISION_IMAGE_FEATURES_HPP
