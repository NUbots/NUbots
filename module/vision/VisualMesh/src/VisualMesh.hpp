#ifndef MODULE_VISION_VISUALMESH_HPP
#define MODULE_VISION_VISUALMESH_HPP

#include <nuclear>

#define CL_TARGET_OPENCL_VERSION 120

#include "engine/opencl/opencl_engine.hpp"
#include "visualmesh.hpp"

namespace module {
namespace vision {

    class VisualMesh : public NUClear::Reactor {
    private:
        template <typename Scalar>
        using Engine     = visualmesh::engine::opencl::Engine<Scalar>;
        using Classifier = visualmesh::engine::opencl::Classifier<float>;
        using VM         = visualmesh::VisualMesh<float, Engine>;

        std::unique_ptr<VM> mesh;
        std::unique_ptr<Classifier> classifier;

    public:
        /// @brief Called by the powerplant to build and setup the VisualMesh reactor.
        explicit VisualMesh(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_VISUALMESH_HPP
