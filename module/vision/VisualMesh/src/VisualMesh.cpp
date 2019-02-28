#include "VisualMesh.h"

#include <Eigen/Geometry>

#include "extension/Configuration.h"

#include "mesh/Sphere.hpp"

#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/VisualMesh.h"

#include "utility/nusight/NUhelpers.h"
#include "utility/support/Timer.hpp"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::Image;
    using message::input::Sensors;
    using message::support::FieldDescription;

    using VisualMeshMsg = message::vision::VisualMesh;

    VisualMesh::VisualMesh(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), mesh_ptr(nullptr) {

        on<Configuration>("VisualMesh.yaml").then([this](const Configuration& config) {
            log("Loading visual mesh");
            network.clear();

            // Load our weights and biases
            for (const auto& conv : config["network"].config) {

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

            draw_mesh   = config["debug"]["draw_mesh"].as<bool>();
            colour_type = config["debug"]["colour_type"].as<int>();

            log("Finished loading visual mesh");
        });

        on<Trigger<Image>, With<FieldDescription>, Buffer<4>>().then([this](const Image& img,
                                                                            const FieldDescription& field) {
            // TODO: Recreate mesh when network configuration changes
            if (!mesh_ptr) {
                mesh_ptr =
                    std::make_unique<mesh::VisualMesh<float>>(mesh::Sphere<float>(0, field.ball_radius, 4, 10),
                                                              0.5,
                                                              1.0,
                                                              50,
                                                              img.lens.fov.maxCoeff() / img.dimensions.maxCoeff());
                // Make the classifier
                classifier = mesh_ptr->make_classifier(network);
            }

            // Get our camera to world matrix
            Eigen::Affine3f Hcw(img.Hcw.cast<float>());

            // Transpose and store in our row major array
            std::array<std::array<float, 4>, 4> Hoc;
            Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(Hoc[0].data()) = Hcw.inverse().matrix();

            mesh::VisualMesh<float>::Lens lens;

            switch (img.lens.projection.value) {
                case Image::Lens::Projection::RECTILINEAR:
                    lens.projection = mesh::VisualMesh<float>::Lens::Projection::RECTILINEAR;
                    break;
                case Image::Lens::Projection::EQUIDISTANT:
                    lens.projection = mesh::VisualMesh<float>::Lens::Projection::EQUIDISTANT;
                    break;
                case Image::Lens::Projection::EQUISOLID:
                    lens.projection = mesh::VisualMesh<float>::Lens::Projection::EQUISOLID;
                    break;
                default: log<NUClear::WARN>("Unknown lens projection."); return;
            }
            lens.dimensions   = std::array<int, 2>{img.dimensions.x(), img.dimensions.y()};
            lens.fov          = img.lens.fov.maxCoeff();
            lens.focal_length = img.lens.focal_length;

            auto results = classifier(img.data.data(), mesh::VisualMesh<float>::FOURCC(img.format), Hoc, lens);

            // Get the mesh that was used so we can make our message
            const auto& m = mesh_ptr->height(Hoc[2][3]);

            auto msg = std::make_unique<VisualMeshMsg>();

            // Pass through our camera id
            msg->camera_id = img.camera_id;

            // Add our description
            for (const auto& r : m.rows) {
                msg->mesh.emplace_back(r.phi, r.end - r.begin);
            }

            // Add our indices
            msg->indices = std::move(results.global_indices);

            // Add our neighbourhood
            msg->neighbourhood = Eigen::Map<const Eigen::Matrix<int, Eigen::Dynamic, 6, Eigen::RowMajor>>(
                reinterpret_cast<int*>(results.neighbourhood.data()), results.neighbourhood.size(), 6);

            // Add our classifications
            std::vector<float> classifications = results.classifications.back().second;
            msg->coordinates = Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>>(
                reinterpret_cast<float*>(classifications.data()), classifications.size(), 2);

            // Add our coordinates
            std::vector<std::array<float, 2>> coordinates = results.pixel_coordinates;
            msg->coordinates = Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>>(
                reinterpret_cast<float*>(coordinates.data()), coordinates.size(), 2);

            if (draw_mesh) {
                std::vector<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>,
                            Eigen::aligned_allocator<std::tuple<Eigen::Vector2i, Eigen::Vector2i, Eigen::Vector4d>>>
                    lines;

                std::vector<float> classification;
                if (colour_type == 0) {
                    classification = results.classifications.front().second;
                }
                if (colour_type == 1) {
                    classification = results.classifications.back().second;
                }

                for (uint i = 0; i < msg->coordinates.rows(); ++i) {

                    Eigen::Vector2f p1(msg->coordinates.row(i));

                    Eigen::Vector4d colour;
                    if (colour_type == 1) {
                        colour << double(classification[i * 2 + 1] > 0.5), 0.0, double(classification[i * 2 + 0] > 0.5),
                            1.0;
                    }

                    if (colour_type == 0) {
                        colour << classification[i * 4 + 0], classification[i * 4 + 1], classification[i * 4 + 2], 1.0;
                    }

                    for (const auto& n : results.neighbourhood[i]) {
                        if (n < msg->coordinates.rows()) {
                            Eigen::Vector2f p2(msg->coordinates.row(n));
                            Eigen::Vector2f p2x = p1 + ((p2 - p1) / 2);
                            lines.emplace_back(p1.cast<int>(), p2x.cast<int>(), colour);
                        }
                    }
                }
                emit(utility::nusight::drawVisionLines(lines));
            }
            emit(msg);
        });
    }
}  // namespace vision
}  // namespace module
