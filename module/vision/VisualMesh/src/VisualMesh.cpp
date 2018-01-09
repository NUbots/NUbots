#include "VisualMesh.h"

#include <Eigen/Geometry>

#include "extension/Configuration.h"
#include "mesh/Sphere.hpp"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/Timer.hpp"

namespace module {
namespace vision {

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), mesh(mesh::Sphere<float>(0, 0.075, 4, 10), 0.5, 1.0, 50, M_PI / 1280.0) {

        on<Configuration>("VisualMesh.yaml").then([this](const Configuration& config) {

            log("Loading visual mesh");

            // Build our classification network
            std::vector<std::vector<std::pair<std::vector<std::vector<float>>, std::vector<float>>>> network;

            // Load our weights and biases
            for (const auto& conv : config.config) {

                // New conv layer
                network.emplace_back();
                auto& net_conv = network.back();

                for (const auto& layer : conv) {

                    // New network layer
                    net_conv.emplace_back();
                    auto& net_layer = net_conv.back();

                    // Copy across our weights
                    for (const auto& l : layer["weights"]) {
                        net_layer.first.emplace_back();
                        auto& weight = net_layer.first.back();

                        for (const auto& v : l) {
                            weight.push_back(v.as<float>());
                        }
                    }

                    // Copy across our biases
                    for (const auto& v : layer["biases"]) {
                        net_layer.second.push_back(v.as<float>());
                    }
                }
            }

            // Make the classifier
            classifier = mesh.make_classifier(network);

            log("Finished loading visual mesh");
        });

        on<Trigger<Image>, Buffer<2>>().then([this](const Image& img) {

            Eigen::Affine3f Hcw(img.Hcw.cast<float>());

            // Transpose Hcw into Hoc
            std::array<std::array<float, 4>, 4> Hoc;

            Eigen::Map<Eigen::Matrix4f>(Hoc[0].data()) = Hcw.inverse().matrix();

            mesh::VisualMesh<float>::Lens lens;
            lens.projection   = mesh::VisualMesh<float>::Lens::EQUIDISTANT;
            lens.dimensions   = {{int(img.dimensions[0]), int(img.dimensions[1])}};
            lens.fov          = M_PI;
            lens.focal_length = (1.0 / 0.0026997136600899543);

            Timer t;
            auto projected = mesh.project(Hoc, lens);
            auto results   = classifier(img.data.data(), mesh::VisualMesh<float>::FOURCC(img.format), Hoc, lens);

            std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>,
                        Eigen::aligned_allocator<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>>>
                lines;


            std::vector<std::array<int, 2>> pixel_coordinates = projected.pixel_coordinates;

            for (uint i = 0; i < pixel_coordinates.size(); ++i) {

                Eigen::Vector2i p1(pixel_coordinates[i][0], pixel_coordinates[i][1]);

                Eigen::Vector4d colour(results[i][1], 0, results[i][0], 1);

                for (const auto& n : projected.neighbourhood[i]) {
                    if (n < pixel_coordinates.size()) {
                        Eigen::Vector2i p2(pixel_coordinates[n][0], pixel_coordinates[n][1]);
                        Eigen::Vector2i p2x = p1 + ((p2 - p1) / 2);
                        lines.emplace_back(p1, p2x, colour);
                    }
                }
            }
            emit(utility::nubugger::drawVisionLines(lines));

            t.measure("Finished processing");
        });

        on<Shutdown>().then([this] {

        });
    }
}  // namespace vision
}  // namespace module
