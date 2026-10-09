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
#include "ONNXRuntime.hpp"

#include <nuclear>
#include <onnxruntime_cxx_api.h>
#include <stdexcept>

namespace utility::onnx {

    namespace {

        /// Forward ONNX Runtime's log messages to NUClear's logger instead of stderr
        void log_to_nuclear(void* /*param*/,
                            OrtLoggingLevel severity,
                            const char* /*category*/,
                            const char* /*logid*/,
                            const char* /*code_location*/,
                            const char* message) {
            switch (severity) {
                case ORT_LOGGING_LEVEL_FATAL:
                case ORT_LOGGING_LEVEL_ERROR: NUClear::log<NUClear::LogLevel::ERROR>(message); break;
                case ORT_LOGGING_LEVEL_WARNING: NUClear::log<NUClear::LogLevel::WARN>(message); break;
                case ORT_LOGGING_LEVEL_INFO: NUClear::log<NUClear::LogLevel::INFO>(message); break;
                case ORT_LOGGING_LEVEL_VERBOSE: NUClear::log<NUClear::LogLevel::DEBUG>(message); break;
            }
        }

        /// Dynamic dimensions (reported as -1) are assumed to be a batch size of 1
        std::vector<int64_t> resolve_shape(const std::vector<int64_t>& shape) {
            std::vector<int64_t> resolved = shape;
            for (int64_t& dim : resolved) {
                if (dim < 0) {
                    dim = 1;
                }
            }
            return resolved;
        }

        size_t element_count(const std::vector<int64_t>& shape) {
            size_t count = 1;
            for (int64_t dim : shape) {
                count *= size_t(dim);
            }
            return count;
        }

        /// ONNX Runtime expects a single Env per process, shared across all sessions
        Ort::Env& shared_env() {
            static Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "NUbots", &log_to_nuclear, nullptr};
            return env;
        }
    }  // namespace

    struct ONNXRuntime::Impl {
        Ort::Session session{nullptr};
        Ort::MemoryInfo memory_info{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
        std::string input_name{};
        std::string output_name{};
        std::vector<int64_t> input_shape{};
        std::vector<int64_t> output_shape{};
        size_t input_count  = 0;
        size_t output_count = 0;
    };

    ONNXRuntime::ONNXRuntime(const std::string& onnx_path, const std::string& device_type)
        : impl(std::make_unique<Impl>()) {
        // The Env registers ORT's default logger, which loading the TensorRT EP below logs through
        Ort::Env& env = shared_env();
        Ort::SessionOptions session_options{};

        // ORT always falls back to its built-in CPU EP, so CPU needs no provider registered
        if (device_type == "gpu") {
            const OrtApi& api                 = Ort::GetApi();
            OrtTensorRTProviderOptionsV2* trt = nullptr;
            Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&trt));
            std::unique_ptr<OrtTensorRTProviderOptionsV2, decltype(api.ReleaseTensorRTProviderOptions)> trt_guard(
                trt,
                api.ReleaseTensorRTProviderOptions);
            Ort::ThrowOnError(api.UpdateTensorRTProviderOptions(trt, option_keys.data(), option_values.data(), option_keys.size()));
            session_options.AppendExecutionProvider_TensorRT_V2(*trt);
        }
        else if (device_type != "cpu") {
            throw std::runtime_error("Unknown device type '" + device_type + "', expected cpu or gpu");
        }

        impl->session = Ort::Session(env, onnx_path.c_str(), session_options);

        if (impl->session.GetInputCount() != 1 || impl->session.GetOutputCount() != 1) {
            throw std::runtime_error("Expected a model with exactly one input and one output tensor");
        }

        Ort::AllocatorWithDefaultOptions allocator;
        impl->input_name  = impl->session.GetInputNameAllocated(0, allocator).get();
        impl->output_name = impl->session.GetOutputNameAllocated(0, allocator).get();

        impl->input_shape =
            resolve_shape(impl->session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape());
        impl->output_shape =
            resolve_shape(impl->session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape());
        impl->input_count  = element_count(impl->input_shape);
        impl->output_count = element_count(impl->output_shape);
    }

    ONNXRuntime::~ONNXRuntime() = default;

    const std::vector<int64_t>& ONNXRuntime::input_shape() const {
        return impl->input_shape;
    }

    const std::vector<int64_t>& ONNXRuntime::output_shape() const {
        return impl->output_shape;
    }

    std::vector<float> ONNXRuntime::infer(const std::vector<float>& input) {
        if (input.size() != impl->input_count) {
            throw std::runtime_error("Input size " + std::to_string(input.size()) + " does not match model input "
                                     + std::to_string(impl->input_count));
        }

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(impl->memory_info,
                                                                   const_cast<float*>(input.data()),
                                                                   input.size(),
                                                                   impl->input_shape.data(),
                                                                   impl->input_shape.size());

        const char* input_names[]  = {impl->input_name.c_str()};
        const char* output_names[] = {impl->output_name.c_str()};

        auto output_tensors =
            impl->session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

        const float* data = output_tensors.front().GetTensorData<float>();
        return std::vector<float>(data, data + impl->output_count);
    }

}  // namespace utility::onnx
