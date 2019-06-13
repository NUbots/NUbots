#include "VisualMesh.h"

#include <Eigen/Geometry>

#include "extension/Configuration.h"

#include "mesh/Sphere.hpp"

#include "message/input/CameraParameters.h"
#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/VisualMesh.h"

#include "utility/nusight/NUhelpers.h"
#include "utility/support/Timer.hpp"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::input::CameraParameters;
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

        // TODO: Recreate mesh when network configuration changes
        on<Startup, With<FieldDescription>, With<CameraParameters>>().then([this](const FieldDescription& field,
                                                                                  const CameraParameters& cam) {
            mesh_ptr = std::make_unique<mesh::VisualMesh<float>>(mesh::Sphere<float>(0, field.ball_radius, 4, 10),
                                                                 0.5,
                                                                 1.0,
                                                                 50,
                                                                 cam.FOV.maxCoeff() / cam.imageSizePixels.maxCoeff());
            // Make the classifier
            classifier = mesh_ptr->make_classifier(network);
        });

        on<Trigger<Image>, With<CameraParameters>, Buffer<4>>().then([this](const Image& img,
                                                                            const CameraParameters& cam) {
            // Get our camera to world matrix
            Eigen::Affine3f Hcw(img.Hcw.cast<float>());

            // Transpose and store in our row major array
            std::array<std::array<float, 4>, 4> Hoc;
            Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(Hoc[0].data()) = Hcw.inverse().matrix();

            mesh::VisualMesh<float>::Lens lens;
            switch (cam.lens.value) {
                case CameraParameters::LensType::RADIAL:
                    lens.projection   = mesh::VisualMesh<float>::Lens::EQUIDISTANT;
                    lens.focal_length = 1.0 / cam.radial.radiansPerPixel;
                    break;
                case CameraParameters::LensType::PINHOLE:
                    lens.projection   = mesh::VisualMesh<float>::Lens::RECTILINEAR;
                    lens.focal_length = cam.pinhole.focalLengthPixels;
                    break;
            }
            lens.dimensions = {{int(img.dimensions[0]), int(img.dimensions[1])}};
            lens.fov        = cam.FOV.x();

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
            msg->neighbourhood.reserve(results.neighbourhood.size());
            for (const auto& n : results.neighbourhood) {
                msg->neighbourhood.emplace_back(Eigen::Map<const Eigen::Matrix<int, 6, 1, Eigen::DontAlign>>(n.data()));
            }

            // Add our classifications
            // Add our first (the image) and last (the results) to our list
            for (const auto& c : results.classifications) {
                msg->classifications.emplace_back(c.first, c.second);
            }


            // -- Graphing for NUsight
            msg->classifications.emplace_back(results.classifications.front().first,
                                              results.classifications.front().second);
            msg->classifications.emplace_back(results.classifications.back().first,
                                              results.classifications.back().second);

            // Add our coordinates
            std::vector<std::array<float, 2>> pixel_coordinates = results.pixel_coordinates;
            msg->coordinates.reserve(pixel_coordinates.size());
            for (const auto& coord : pixel_coordinates) {
                msg->coordinates.push_back({int(coord[0]), int(coord[1])});
            }

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

                for (uint i = 0; i < msg->coordinates.size(); ++i) {

                    Eigen::Vector2i p1(msg->coordinates[i]);

                    Eigen::Vector4d colour;
                    if (colour_type == 1) {
                        colour << double(classification[i * 2 + 1] > 0.5), 0.0, double(classification[i * 2 + 0] > 0.5),
                            1.0;
                    }

                    if (colour_type == 0) {
                        colour << classification[i * 4 + 0], classification[i * 4 + 1], classification[i * 4 + 2], 1.0;
                    }

                    for (const auto& n : results.neighbourhood[i]) {
                        if (n < msg->coordinates.size()) {
                            Eigen::Vector2i p2(msg->coordinates[n]);
                            Eigen::Vector2i p2x = p1 + ((p2 - p1) / 2);
                            lines.emplace_back(p1, p2x, colour);
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
