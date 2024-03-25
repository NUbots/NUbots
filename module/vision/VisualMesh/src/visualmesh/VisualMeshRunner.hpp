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
#ifndef MODULE_VISION_VISUALMESH_VISUALMESHRUNNER_HPP
#define MODULE_VISION_VISUALMESH_VISUALMESHRUNNER_HPP

#define CL_TARGET_OPENCL_VERSION 120
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "message/input/Image.hpp"

namespace module::vision::visualmesh {

    struct VisualMeshResults {
        Eigen::Matrix<float, 3, Eigen::Dynamic> rays;
        Eigen::Matrix<float, 2, Eigen::Dynamic> coordinates;
        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> neighbourhood;
        std::vector<int> indices;
        Eigen::MatrixXf classifications;
    };

    class VisualMeshRunner {
    private:
        int n_neighbours = 0;
        std::function<VisualMeshResults(const message::input::Image&, const Eigen::Isometry3f&)> runner;

    public:
        VisualMeshRunner(const std::string& engine,
                         const double& min_height,
                         const double& max_height,
                         const double& max_distance,
                         const double& intersection_tolerance,
                         const std::string& path,
                         const std::string& cache_directory);
        VisualMeshResults operator()(const message::input::Image& image, const Eigen::Isometry3f& Htc);

        /// Map of class names to class indices
        std::map<std::string, uint32_t> class_map;
    };

}  // namespace module::vision::visualmesh

#endif  // MODULE_VISION_VISUALMESH_VISUALMESHRUNNER_HPP
