#include "VisualMesh.h"

#include "extension/Configuration.h"
#include "mesh/Sphere.hpp"
#include "mesh/Timer.hpp"
#include "message/input/Image.h"
#include "message/input/Sensors.h"

namespace module {
namespace vision {

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), mesh(mesh::Sphere<float>(0, 0.075, 4, 10), 0.5, 1.0, 50, M_PI / 1280.0) {

        on<Configuration>("VisualMesh.yaml").then([this](const Configuration& config) {
            // Use configuration here from file VisualMesh.yaml
        });

        on<Trigger<Image>, Single>().then([this](const Image& img) {

            Eigen::Matrix4d Hwc = img.Hcw.inverse();

            // Transpose Hcw into Hoc
            std::array<std::array<float, 4>, 4> Hoc;
            Hoc[0][0] = Hwc(0, 0);
            Hoc[0][1] = Hwc(0, 1);
            Hoc[0][2] = Hwc(0, 2);
            Hoc[0][3] = Hwc(0, 3);
            Hoc[1][0] = Hwc(1, 0);
            Hoc[1][1] = Hwc(1, 1);
            Hoc[1][2] = Hwc(1, 2);
            Hoc[1][3] = Hwc(1, 3);
            Hoc[2][0] = Hwc(2, 0);
            Hoc[2][1] = Hwc(2, 1);
            Hoc[2][2] = Hwc(2, 2);
            Hoc[2][3] = Hwc(2, 3);
            Hoc[3][0] = Hwc(3, 0);
            Hoc[3][1] = Hwc(3, 1);
            Hoc[3][2] = Hwc(3, 2);
            Hoc[3][3] = Hwc(3, 3);

            log(Hoc[0][3], Hoc[1][3], Hoc[2][3]);

            mesh::VisualMesh<float>::Lens lens;
            lens.type                     = mesh::VisualMesh<float>::Lens::RADIAL;
            lens.dimensions               = {{img.dimensions[0], img.dimensions[1]}};
            lens.radial.fov               = M_PI;
            lens.radial.pixels_per_radian = 1.0 / 0.00245;

            Timer t;
            mesh.classify(img.data.data(), img.data.size(), mesh::VisualMesh<float>::FOURCC(img.format), Hoc, lens);
            t.measure("Finished processing");
        });

        on<Shutdown>().then([this] {

        });
    }
}  // namespace vision
}  // namespace module
