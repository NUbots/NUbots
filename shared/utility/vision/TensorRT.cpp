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
#include "TensorRT.hpp"

#include <cuda_runtime.h>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "NvInfer.h"
#include "NvOnnxParser.h"

using namespace nvinfer1;

namespace utility::vision {

    namespace {

        /// Logger for TensorRT that only reports warnings and errors
        class Logger : public ILogger {
            void log(Severity severity, char const* msg) noexcept override {
                if (severity <= Severity::kWARNING) {
                    std::cout << msg << std::endl;
                }
            }
        };
        Logger logger;

        void cuda_check(cudaError_t err, const char* what) {
            if (err != cudaSuccess) {
                throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(err));
            }
        }

        /// Build a serialized engine plan from an ONNX model
        std::vector<char> build_plan(const std::string& onnx_path, bool fp16) {
            auto builder = std::unique_ptr<IBuilder>(createInferBuilder(logger));
            auto network = std::unique_ptr<INetworkDefinition>(builder->createNetworkV2(0));
            auto parser  = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger));
            if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int>(ILogger::Severity::kWARNING))) {
                throw std::runtime_error("Failed to parse ONNX model: " + onnx_path);
            }

            auto config = std::unique_ptr<IBuilderConfig>(builder->createBuilderConfig());
            if (fp16 && builder->platformHasFastFp16()) {
                config->setFlag(BuilderFlag::kFP16);
            }

            auto plan = std::unique_ptr<IHostMemory>(builder->buildSerializedNetwork(*network, *config));
            if (plan == nullptr) {
                throw std::runtime_error("Failed to build TensorRT engine from: " + onnx_path);
            }
            auto* data = static_cast<char*>(plan->data());
            return std::vector<char>(data, data + plan->size());
        }

        std::vector<char> read_file(const std::string& path) {
            std::ifstream file(path, std::ios::binary | std::ios::ate);
            if (!file.is_open()) {
                return {};
            }
            std::vector<char> bytes(file.tellg());
            file.seekg(0, std::ios::beg);
            if (!file.read(bytes.data(), std::streamsize(bytes.size()))) {
                return {};
            }
            return bytes;
        }

    }  // namespace

    struct TensorRT::Impl {
        std::unique_ptr<IRuntime> runtime{};
        std::unique_ptr<ICudaEngine> engine{};
        std::unique_ptr<IExecutionContext> context{};
        std::vector<int64_t> input_shape{};
        std::vector<int64_t> output_shape{};
        size_t input_count    = 0;
        size_t output_count   = 0;
        void* d_input         = nullptr;
        void* d_output        = nullptr;
        cudaStream_t stream   = nullptr;

        ~Impl() {
            // Tear down the execution context before freeing the memory it may reference
            context.reset();
            cudaFree(d_input);
            cudaFree(d_output);
            if (stream != nullptr) {
                cudaStreamDestroy(stream);
            }
        }
    };

    TensorRT::TensorRT(const std::string& onnx_path, bool fp16) : impl(std::make_unique<Impl>()) {
        // Engine plans are only valid for this GPU and TensorRT build, so cache per TensorRT version
        const std::string plan_path =
            onnx_path + ".trt" + std::to_string(getInferLibVersion()) + (fp16 ? ".fp16" : ".fp32") + ".plan";

        std::vector<char> plan = read_file(plan_path);
        bool from_cache        = !plan.empty();

        auto save_plan = [&] {
            // Caching is best effort, if the directory is read only we just rebuild next time
            std::ofstream out(plan_path, std::ios::binary);
            out.write(plan.data(), std::streamsize(plan.size()));
            if (!out) {
                std::cout << "Failed to cache TensorRT plan to " << plan_path
                          << ", the engine will be rebuilt every startup" << std::endl;
            }
        };

        if (!from_cache) {
            plan = build_plan(onnx_path, fp16);
            save_plan();
        }

        impl->runtime.reset(createInferRuntime(logger));
        impl->engine.reset(impl->runtime->deserializeCudaEngine(plan.data(), plan.size()));

        // A cached plan can be stale (e.g. built by a different TensorRT patch or for a different GPU)
        if (impl->engine == nullptr && from_cache) {
            plan = build_plan(onnx_path, fp16);
            save_plan();
            impl->engine.reset(impl->runtime->deserializeCudaEngine(plan.data(), plan.size()));
        }
        if (impl->engine == nullptr) {
            throw std::runtime_error("Failed to deserialize TensorRT engine for: " + onnx_path);
        }

        impl->context.reset(impl->engine->createExecutionContext());
        if (impl->context == nullptr) {
            throw std::runtime_error("Failed to create TensorRT execution context");
        }

        if (impl->engine->getNbIOTensors() != 2) {
            throw std::runtime_error("Expected a model with exactly one input and one output tensor");
        }
        const char* input_name  = impl->engine->getIOTensorName(0);
        const char* output_name = impl->engine->getIOTensorName(1);

        auto count_elements = [](const Dims& dims, const char* name) {
            size_t count = 1;
            for (int32_t i = 0; i < dims.nbDims; ++i) {
                if (dims.d[i] < 0) {
                    throw std::runtime_error(std::string("Dynamic shapes are not supported (tensor ") + name + ")");
                }
                count *= size_t(dims.d[i]);
            }
            return count;
        };

        const Dims input_dims  = impl->engine->getTensorShape(input_name);
        const Dims output_dims = impl->engine->getTensorShape(output_name);
        impl->input_count      = count_elements(input_dims, input_name);
        impl->output_count     = count_elements(output_dims, output_name);
        impl->input_shape.assign(input_dims.d, input_dims.d + input_dims.nbDims);
        impl->output_shape.assign(output_dims.d, output_dims.d + output_dims.nbDims);

        cuda_check(cudaMalloc(&impl->d_input, impl->input_count * sizeof(float)), "cudaMalloc input");
        cuda_check(cudaMalloc(&impl->d_output, impl->output_count * sizeof(float)), "cudaMalloc output");
        cuda_check(cudaStreamCreate(&impl->stream), "cudaStreamCreate");
        impl->context->setTensorAddress(input_name, impl->d_input);
        impl->context->setTensorAddress(output_name, impl->d_output);
    }

    TensorRT::~TensorRT() = default;

    const std::vector<int64_t>& TensorRT::input_shape() const {
        return impl->input_shape;
    }

    const std::vector<int64_t>& TensorRT::output_shape() const {
        return impl->output_shape;
    }

    std::vector<float> TensorRT::infer(const std::vector<float>& input) {
        if (input.size() != impl->input_count) {
            throw std::runtime_error("Input size " + std::to_string(input.size()) + " does not match model input "
                                     + std::to_string(impl->input_count));
        }

        std::vector<float> output(impl->output_count);
        cuda_check(
            cudaMemcpyAsync(impl->d_input, input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice, impl->stream),
            "copy input to device");
        if (!impl->context->enqueueV3(impl->stream)) {
            throw std::runtime_error("enqueueV3 failed");
        }
        cuda_check(
            cudaMemcpyAsync(output.data(), impl->d_output, output.size() * sizeof(float), cudaMemcpyDeviceToHost, impl->stream),
            "copy output to host");
        cuda_check(cudaStreamSynchronize(impl->stream), "cudaStreamSynchronize");
        return output;
    }

}  // namespace utility::vision
