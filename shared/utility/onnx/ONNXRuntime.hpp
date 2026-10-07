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
#ifndef UTILITY_ONNX_ONNXRUNTIME_HPP
#define UTILITY_ONNX_ONNXRUNTIME_HPP

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace utility::onnx {

    /// Runs inference with ONNX Runtime on a single-input, single-output model
    class ONNXRuntime {
    public:
        /// Load an ONNX model for inference
        /// @param onnx_path Path to the ONNX model file
        /// @param device_type "cpu", or "gpu" to use the TensorRT execution provider
        /// @param fp16 Let the TensorRT execution provider run in FP16 precision (ignored on "cpu")
        /// @throws std::runtime_error if the model cannot be loaded or device_type is unknown
        explicit ONNXRuntime(const std::string& onnx_path, const std::string& device_type, bool fp16 = true);
        ~ONNXRuntime();

        ONNXRuntime(const ONNXRuntime&)            = delete;
        ONNXRuntime& operator=(const ONNXRuntime&) = delete;

        /// Shape of the model's input tensor, e.g. {1, 3, 640, 640}
        [[nodiscard]] const std::vector<int64_t>& input_shape() const;

        /// Shape of the model's output tensor, e.g. {1, 10, 8400}
        [[nodiscard]] const std::vector<int64_t>& output_shape() const;

        /// Run inference on the model
        /// @param input Input tensor data, must match the input tensor's element count
        /// @return Output tensor data
        std::vector<float> infer(const std::vector<float>& input);

    private:
        struct Impl;
        std::unique_ptr<Impl> impl;
        // The API for the TensorRT EP takes in option keys and values as separate arrays
        // Configuration options are available at:
        // https://onnxruntime.ai/docs/execution-providers/TensorRT-ExecutionProvider.html#configurations
        std::vector<const char*> option_keys = {
            "trt_fp16_enable",
            "trt_engine_cache_enable",
            "trt_engine_cache_path",
            "trt_timing_cache_enable",
            "trt_timing_cache_path",
        };
        std::vector<const char*> option_values = {
            "1",
            "1",
            "/tmp/onnx_trt_cache",
            "1",
            "/tmp/onnx_trt_cache",
        };
    };

}  // namespace utility::onnx

#endif  // UTILITY_ONNX_ONNXRUNTIME_HPP
