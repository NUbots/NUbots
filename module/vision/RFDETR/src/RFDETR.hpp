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
#ifndef MODULE_VISION_RFDETR_HPP
#define MODULE_VISION_RFDETR_HPP

#include <Eigen/Core>
#include <nuclear>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

namespace module::vision {

    class RFDETR : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            /// @brief Global minimum score (sigmoid of logit) for a detection to be considered at all
            double score_threshold = 0.3;
            /// @brief Class-channel offset (logit channel = class_id + offset). -1 means auto-detect:
            /// RF-DETR emits num_classes = objects + 1 channels (channel 0 is an unused background slot),
            /// so the offset is 1 when the logit dimension is one larger than the number of objects.
            int class_offset = -1;
        } cfg;

        /// @brief OpenVINO compiled model, used to create inference request object
        ov::CompiledModel compiled_model{};

        /// @brief Inference request, used to run the model (inference)
        ov::InferRequest infer_request{};

        /// @brief ImageNet normalisation applied after scaling to [0, 1]. RGB channel order.
        /// MUST match the training/export preprocessing (see ../training/verify_onnx.py).
        const cv::Vec3f imagenet_mean = {0.485f, 0.456f, 0.406f};
        const cv::Vec3f imagenet_std  = {0.229f, 0.224f, 0.225f};

        /// @brief Object struct for storing name and colour
        struct Object {
            /// @brief Class name
            std::string name = "";
            /// @brief Colour for bounding box
            Eigen::Vector4d colour = Eigen::Vector4d(1, 1, 1, 1);
            /// @brief Confidence threshold
            double confidence_threshold = 0.0;
        };

        /// @brief The objects that the RF-DETR model can detect. The order is fixed and must match
        /// the training class order (see ../training/convert_to_coco.py CANONICAL_CLASSES).
        std::vector<Object> objects = {{"ball", Eigen::Vector4d(1, 1, 1, 1), 0.0},
                                       {"goal post", Eigen::Vector4d(1, 0, 1, 1), 0.0},
                                       {"robot", Eigen::Vector4d(1, 0.5, 0, 1), 0.0},
                                       {"L-intersection", Eigen::Vector4d(1, 0, 0, 1), 0.0},
                                       {"T-intersection", Eigen::Vector4d(0, 1, 0, 1), 0.0},
                                       {"X-intersection", Eigen::Vector4d(0, 0, 1, 1), 0.0}};


    public:
        /// @brief Called by the powerplant to build and setup the RFDETR reactor.
        explicit RFDETR(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision

#endif  // MODULE_VISION_RFDETR_HPP
