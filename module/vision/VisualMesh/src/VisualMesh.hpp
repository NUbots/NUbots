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
        std::map<std::string, std::vector<visualmesh::VisualMeshRunner>> engines;

    public:
        /// @brief Called by the powerplant to build and setup the VisualMesh reactor.
        explicit VisualMesh(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::vision
#endif  // MODULE_VISION_VISUALMESH_HPP
