/*
 * MIT License
 *
 * Copyright (c) 2018 NUbots
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
#ifndef MODULE_VISION_VISUALMESH_HPP
#define MODULE_VISION_VISUALMESH_HPP

#include <map>
#include <nuclear>
#include <string>
#include <vector>

#include "visualmesh/VisualMeshRunner.hpp"

namespace module::vision {

    class VisualMesh : public NUClear::Reactor {
    private:
        struct EngineContext {
            struct Runner {
                /// Mutex controlling access to this runner instance
                std::unique_ptr<std::mutex> mutex;
                /// The actual runner that will process the image
                visualmesh::VisualMeshRunner runner;
            };

            /// A list of compressors that can be used
            std::vector<Runner> runners;
        };

    public:
        /// @brief Called by the powerplant to build and setup the VisualMesh reactor.
        explicit VisualMesh(std::unique_ptr<NUClear::Environment> environment);

    private:
        /// Mutex to protect access to the engines map
        std::mutex engines_mutex;
        /// The engines that are available for use
        std::map<std::string, std::shared_ptr<EngineContext>> engines;

        /// Number of images that have been processed since this was last reset
        int processed = 0;
        /// Number of images that have been dropped since this was last reset
        int dropped = 0;

        /// Cumulative duration of processed frames in milliseconds
        double cumulative_duration_ms = 0.0;
        /// Number of processed frames for average FPS calculation
        int frame_count = 0;
    };

}  // namespace module::vision
#endif  // MODULE_VISION_VISUALMESH_HPP
