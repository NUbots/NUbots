#include "VisualMesh.h"

#include <Eigen/Geometry>

#include "extension/Configuration.h"
#include "mesh/Sphere.hpp"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/vision/VisualMesh.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/Timer.hpp"

namespace module {
namespace vision {

    using extension::Configuration;
    using message::input::Image;
    using message::input::Sensors;
    using VisualMeshMsg = message::vision::VisualMesh;

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

        on<Trigger<Image>, Buffer<4>>().then([this](const Image& img) {

            // Get our camera to world matrix
            Eigen::Affine3f Hcw(img.Hcw.cast<float>());

            // Transpose and store in our row major array
            std::array<std::array<float, 4>, 4> Hoc;
            Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(Hoc[0].data()) = Hcw.inverse().matrix();

            mesh::VisualMesh<float>::Lens lens;
            lens.projection   = mesh::VisualMesh<float>::Lens::EQUIDISTANT;
            lens.dimensions   = {{int(img.dimensions[0]), int(img.dimensions[1])}};
            lens.fov          = M_PI;
            lens.focal_length = (1.0 / 0.0026997136600899543);

            Timer t;
            auto results = classifier(img.data.data(), mesh::VisualMesh<float>::FOURCC(img.format), Hoc, lens);
            t.measure("Classified");

            // Get the mesh that was used so we can make our message
            const auto& m = mesh.height(Hoc[2][3]);

            auto msg = std::make_unique<VisualMeshMsg>();

            // Pass through our camera id
            msg->camera_id = img.camera_id;

            // Add our description
            for (const auto& r : m.rows) {
                msg->rows.emplace_back(r.phi, r.end - r.begin);
            }

            // Add our indices
            msg->indices = std::move(results.global_indices);

            // Add our coordinates
            msg->coordinates = results.pixel_coordinates.as<Eigen::Matrix<int, 2, 1, Eigen::DontAlign>>();

            // Add our classifications
            // Add our first (the image) and last (the results) to our list
            for (const auto& c : results.classifications) {
                msg->classifications.emplace_back(c.first, c.second);
            }

            emit(msg);
            // msg->classifications.emplace_back(results.classifications.front().first,
            //                                   results.classifications.front().second);
            // msg->classifications.emplace_back(results.classifications.back().first,
            //                                   results.classifications.back().second);


            // std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>,
            //             Eigen::aligned_allocator<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>>>
            //     lines;


            // std::vector<std::array<int, 2>> pixel_coordinates = results.pixel_coordinates;
            // std::vector<float> classification                 = results.classifications.front().second;

            // for (uint i = 0; i < pixel_coordinates.size(); ++i) {

            //     Eigen::Vector2i p1(pixel_coordinates[i][0], pixel_coordinates[i][1]);

            //     // Eigen::Vector4d colour(results.second[i][1], 0, results.second[i][0], 1);
            //     // Eigen::Vector4d colour(classification[i * 2 + 1] > 0.5, 0, classification[i * 2 + 0] > 0.5, 1);
            //     Eigen::Vector4d colour(
            //         classification[i * 4 + 0], classification[i * 4 + 1], classification[i * 4 + 2], 1);

            //     for (const auto& n : results.neighbourhood[i]) {
            //         if (n < pixel_coordinates.size()) {
            //             Eigen::Vector2i p2(pixel_coordinates[n][0], pixel_coordinates[n][1]);
            //             Eigen::Vector2i p2x = p1 + ((p2 - p1) / 2);
            //             lines.emplace_back(p1, p2x, colour);
            //         }
            //     }
            // }
            // emit(utility::nubugger::drawVisionLines(lines));

            t.measure("Saved in message");
        });

        on<Shutdown>().then([this] {

        });
    }
}  // namespace vision
}  // namespace module
