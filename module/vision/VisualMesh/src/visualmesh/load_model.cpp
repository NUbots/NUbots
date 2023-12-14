/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#include "load_model.hpp"

namespace module::vision::visualmesh {

    inline ::visualmesh::ActivationFunction activation_function(const std::string& name) {
        // clang-format off
        if (name == "selu")    { return ::visualmesh::ActivationFunction::SELU;    }
        if (name == "softmax") { return ::visualmesh::ActivationFunction::SOFTMAX; }
        if (name == "relu")    { return ::visualmesh::ActivationFunction::RELU;    }
        if (name == "tanh")    { return ::visualmesh::ActivationFunction::TANH;    }
        // clang-format on
        throw std::runtime_error("Unknown activation function " + name);
    }

    LoadedModel load_model(const std::string& path) {

        ::visualmesh::NetworkStructure<float> model;
        YAML::Node config = YAML::LoadFile(path);
        for (const auto& conv : config["network"]) {
            model.emplace_back();
            auto& net_conv = model.back();

            for (const auto& layer : conv) {
                net_conv.emplace_back(::visualmesh::Layer<float>{
                    layer["weights"].as<std::vector<std::vector<float>>>(),
                    layer["biases"].as<std::vector<float>>(),
                    activation_function(layer["activation"].as<std::string>()),
                });
            }
        }

        LoadedModel loaded;
        loaded.model                  = model;
        loaded.mesh_model             = config["mesh"].as<std::string>();
        loaded.class_map              = config["class_map"].as<std::map<std::string, uint32_t>>();
        loaded.num_classes            = loaded.class_map.size();
        loaded.geometry.shape         = config["geometry"]["shape"].as<std::string>();
        loaded.geometry.radius        = config["geometry"]["radius"].as<double>();
        loaded.geometry.intersections = config["geometry"]["intersections"].as<double>();

        return loaded;
    }


}  // namespace module::vision::visualmesh
