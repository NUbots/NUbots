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
#ifndef UTILITY_VISION_TENSORRT_HPP
#define UTILITY_VISION_TENSORRT_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace utility::vision {

    /// Runs inference with TensorRT on an engine built from an ONNX model.
    ///
    /// Serialized engine plans are only valid for the exact GPU and TensorRT build that created them, so rather
    /// than shipping precompiled plans the engine is built from the ONNX model on the device it will run on. The
    /// resulting plan is cached on disk next to the model (keyed on the TensorRT version), so the build cost is
    /// only paid the first time a model is loaded.
    class TensorRT {
    public:
        /// Build (or load from cache) a TensorRT engine for the given ONNX model.
        /// @param onnx_path Path to the ONNX model file
        /// @param fp16 Build the engine with FP16 precision if the device supports it
        /// @throws std::runtime_error if the model cannot be parsed or the engine cannot be built
        explicit TensorRT(const std::string& onnx_path, bool fp16 = true);
        ~TensorRT();

        TensorRT(const TensorRT&)            = delete;
        TensorRT& operator=(const TensorRT&) = delete;

        /// Shape of the model's input tensor, e.g. {1, 3, 640, 640}
        [[nodiscard]] const std::vector<int64_t>& input_shape() const;

        /// Shape of the model's output tensor, e.g. {1, 10, 8400}
        [[nodiscard]] const std::vector<int64_t>& output_shape() const;

        /// Run inference on the engine
        /// @param input Input tensor data, must match the input tensor's element count
        /// @return Output tensor data
        std::vector<float> infer(const std::vector<float>& input);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
    };

}  // namespace utility::vision

#endif  // UTILITY_VISION_TENSORRT_HPP
