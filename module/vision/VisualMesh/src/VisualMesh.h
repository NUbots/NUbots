#ifndef MODULE_VISION_VISUALMESH_H
#define MODULE_VISION_VISUALMESH_H

#include <nuclear>

#include "mesh/VisualMesh.hpp"

namespace module {
namespace vision {

    class VisualMesh : public NUClear::Reactor {
    private:
        mesh::VisualMesh<float> mesh;
        mesh::VisualMesh<float>::Classifier classifier;

    public:
        /// @brief Called by the powerplant to build and setup the VisualMesh reactor.
        explicit VisualMesh(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_VISUALMESH_H
