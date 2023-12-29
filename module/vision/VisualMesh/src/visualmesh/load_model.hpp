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

#ifndef MODULE_VISION_VISUALMESH_LOAD_MODEL_HPP
#define MODULE_VISION_VISUALMESH_LOAD_MODEL_HPP

#include <map>
#include <string>
#include <visualmesh/network_structure.hpp>
#include <yaml-cpp/yaml.h>

namespace module::vision::visualmesh {

    struct LoadedModel {
        ::visualmesh::NetworkStructure<float> model;
        std::string mesh_model;
        int num_classes = 0;
        std::map<std::string, uint32_t> class_map;
        struct {
            std::string shape;
            double radius        = 0.0;
            double intersections = 0.0;
        } geometry;
    };

    LoadedModel load_model(const std::string& path);

}  // namespace module::vision::visualmesh

#endif  // MODULE_VISION_VISUALMESH_LOAD_MODEL_HPP
