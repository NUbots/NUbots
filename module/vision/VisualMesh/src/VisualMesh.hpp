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
    };

}  // namespace module::vision
#endif  // MODULE_VISION_VISUALMESH_HPP
