#ifndef MODULE_VISION_VISUALMESH_H
#define MODULE_VISION_VISUALMESH_H

#include <nuclear>

#include "mesh/VisualMesh.hpp"

namespace module {
namespace vision {

    class VisualMesh : public NUClear::Reactor {
    private:
        // Build our classification network
        std::vector<std::vector<std::pair<std::vector<std::vector<float>>, std::vector<float>>>> network;

        std::unique_ptr<mesh::VisualMesh<float>> mesh_ptr;
        mesh::VisualMesh<float>::Classifier classifier;

        bool draw_mesh;
        int colour_type;

    public:
        /// @brief Called by the powerplant to build and setup the VisualMesh reactor.
        explicit VisualMesh(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_VISUALMESH_H
