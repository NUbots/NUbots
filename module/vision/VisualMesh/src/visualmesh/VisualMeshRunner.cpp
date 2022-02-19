#include "VisualMeshRunner.hpp"

#include <visualmesh/engine/cpu/engine.hpp>
#include <visualmesh/engine/opencl/engine.hpp>
#include <visualmesh/geometry/Circle.hpp>
#include <visualmesh/geometry/Sphere.hpp>
#include <visualmesh/model/nmgrid4.hpp>
#include <visualmesh/model/nmgrid6.hpp>
#include <visualmesh/model/nmgrid8.hpp>
#include <visualmesh/model/ring4.hpp>
#include <visualmesh/model/ring6.hpp>
#include <visualmesh/model/ring8.hpp>
#include <visualmesh/model/xmgrid4.hpp>
#include <visualmesh/model/xmgrid6.hpp>
#include <visualmesh/model/xmgrid8.hpp>
#include <visualmesh/model/xygrid4.hpp>
#include <visualmesh/model/xygrid6.hpp>
#include <visualmesh/model/xygrid8.hpp>
#include <visualmesh/visualmesh.hpp>
#include <yaml-cpp/yaml.h>

#include "load_model.hpp"

namespace module::vision::visualmesh {

    using message::input::Image;

    struct VisualMeshModelConfig {
        std::string engine;
        ::visualmesh::NetworkStructure<float> model;
        std::string mesh_model;
        int num_classes = 0;

        struct {
            double intersection_tolerance = 0.0;

            struct {
                double min_height;
                double max_height;
                double max_distance;
            } classifier{};

            struct {
                std::string shape;
                double radius        = 0.0;
                double intersections = 0.0;
            } geometry;
        } mesh;
    };

    namespace generate_runner {

        template <template <typename> class Model, template <typename> class Engine, typename Shape>
        std::function<VisualMeshResults(const Image&, const Eigen::Affine3f&)> runner(const VisualMeshModelConfig& cfg,
                                                                                      const Shape& shape) {

            // Make the model and the engine
            auto mesh = std::make_shared<::visualmesh::VisualMesh<float, Model>>(
                ::visualmesh::VisualMesh<double, Model>(shape,
                                                        cfg.mesh.classifier.min_height,
                                                        cfg.mesh.classifier.max_height,
                                                        cfg.mesh.geometry.intersections,
                                                        cfg.mesh.intersection_tolerance,
                                                        cfg.mesh.classifier.max_distance));
            auto engine = std::make_shared<Engine<float>>(cfg.model);

            return [shape, mesh, engine](const Image& img, const Eigen::Affine3f& Hcw) {
                // Create the lens
                ::visualmesh::Lens<float> lens{};
                lens.dimensions   = {int(img.dimensions[0]), int(img.dimensions[1])};
                lens.focal_length = img.lens.focal_length * img.dimensions[0];
                lens.fov          = img.lens.fov;
                lens.centre       = {img.lens.centre[0] * img.dimensions[0], img.lens.centre[1] * img.dimensions[0]};
                lens.k            = std::array<float, 2>{
                    float(img.lens.k[0] * std::pow(img.dimensions[0], 2)),
                    float(img.lens.k[1] * std::pow(img.dimensions[0], 4)),
                };
                switch (img.lens.projection.value) {
                    case Image::Lens::Projection::EQUIDISTANT: lens.projection = ::visualmesh::EQUIDISTANT; break;
                    case Image::Lens::Projection::EQUISOLID: lens.projection = ::visualmesh::EQUISOLID; break;
                    case Image::Lens::Projection::RECTILINEAR: lens.projection = ::visualmesh::RECTILINEAR; break;
                    default: throw std::runtime_error("Unknown lens projection");
                }

                // Convert our orientation matrix
                std::array<std::array<float, 4>, 4> Hoc{};
                Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(Hoc[0].data()) = Hcw.inverse().matrix();

                // Run the network
                const auto& m = mesh->height(Hoc[2][3]);
                auto output   = engine->operator()(m, Hoc, lens, img.data.data(), img.format);

                // Assemble the results
                VisualMeshResults results;

                if (output.global_indices.empty()) {
                    return results;
                }

                // Get all the rays
                results.rays.resize(3, output.global_indices.size());
                int col = 0;
                for (const auto& i : output.global_indices) {
                    results.rays.col(col++) = Eigen::Vector3f(m.nodes[i].ray[0], m.nodes[i].ray[1], m.nodes[i].ray[2]);
                }

                // The pixels that were projected in the mesh
                results.coordinates = Eigen::Map<Eigen::Matrix<float, 2, Eigen::Dynamic>>(
                    reinterpret_cast<float*>(output.pixel_coordinates.data()),
                    2,
                    output.pixel_coordinates.size());

                // The neighbourhood graph
                results.neighbourhood = Eigen::Map<Eigen::Matrix<int, Model<float>::N_NEIGHBOURS, Eigen::Dynamic>>(
                    reinterpret_cast<int*>(output.neighbourhood.data()),
                    Model<float>::N_NEIGHBOURS,
                    output.neighbourhood.size());

                // The indices of the projected pixels in the global mesh
                results.indices = std::move(output.global_indices);

                // The predicted classification of each projected pixel
                results.classifications = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>(
                    reinterpret_cast<float*>(output.classifications.data()),
                    output.classifications.size() / output.neighbourhood.size(),
                    output.neighbourhood.size());

                return results;
            };
        }

        template <template <typename> class Model, typename Shape>
        std::function<VisualMeshResults(const Image&, const Eigen::Affine3f&)> engine(const VisualMeshModelConfig& cfg,
                                                                                      const Shape& shape) {

            // clang-format off
            if (cfg.engine == "opencl") { return runner<Model, ::visualmesh::engine::opencl::Engine>(cfg, shape); }
            if (cfg.engine == "cpu")    { return runner<Model, ::visualmesh::engine::cpu::Engine>(cfg, shape);    }
            // clang-format on
            throw std::runtime_error("Unknown visual mesh engine type " + cfg.engine);
        }

        template <typename Shape>
        std::function<VisualMeshResults(const Image&, const Eigen::Affine3f&)> model(const VisualMeshModelConfig& cfg,
                                                                                     const Shape& shape) {
            // clang-format off
            if (cfg.mesh_model == "RING4")   { return engine<::visualmesh::model::Ring4>(cfg, shape);   }
            if (cfg.mesh_model == "RING6")   { return engine<::visualmesh::model::Ring6>(cfg, shape);   }
            if (cfg.mesh_model == "RING8")   { return engine<::visualmesh::model::Ring8>(cfg, shape);   }
            if (cfg.mesh_model == "XYGRID4") { return engine<::visualmesh::model::XYGrid4>(cfg, shape); }
            if (cfg.mesh_model == "XYGRID6") { return engine<::visualmesh::model::XYGrid6>(cfg, shape); }
            if (cfg.mesh_model == "XYGRID8") { return engine<::visualmesh::model::XYGrid8>(cfg, shape); }
            if (cfg.mesh_model == "XMGRID4") { return engine<::visualmesh::model::XMGrid4>(cfg, shape); }
            if (cfg.mesh_model == "XMGRID6") { return engine<::visualmesh::model::XMGrid6>(cfg, shape); }
            if (cfg.mesh_model == "XMGRID8") { return engine<::visualmesh::model::XMGrid8>(cfg, shape); }
            if (cfg.mesh_model == "NMGRID4") { return engine<::visualmesh::model::NMGrid4>(cfg, shape); }
            if (cfg.mesh_model == "NMGRID6") { return engine<::visualmesh::model::NMGrid6>(cfg, shape); }
            if (cfg.mesh_model == "NMGRID8") { return engine<::visualmesh::model::NMGrid8>(cfg, shape); }
            // clang-format on
            throw std::runtime_error("Unknown visual mesh model type " + cfg.mesh_model);
        }

        inline std::function<VisualMeshResults(const Image&, const Eigen::Affine3f&)> geometry(
            const VisualMeshModelConfig& cfg) {

            // clang-format off
            if (cfg.mesh.geometry.shape == "SPHERE") {
                return model(cfg, ::visualmesh::geometry::Sphere<double>(cfg.mesh.geometry.radius)); }
            if (cfg.mesh.geometry.shape == "CIRCLE") {
                return model(cfg, ::visualmesh::geometry::Circle<double>(cfg.mesh.geometry.radius)); }
            // clang-format on
            throw std::runtime_error("Unknown visual mesh geometry type " + cfg.mesh.geometry.shape);
        }

    }  // namespace generate_runner

    VisualMeshRunner::VisualMeshRunner(const std::string& engine,
                                       const double& min_height,
                                       const double& max_height,
                                       const double& max_distance,
                                       const double& intersection_tolerance,
                                       const std::string& path)
        : active(std::make_unique<std::atomic<bool>>()) {

        // Add the configuration properties we were passed
        VisualMeshModelConfig cfg;
        cfg.engine                       = engine;
        cfg.mesh.intersection_tolerance  = intersection_tolerance;
        cfg.mesh.classifier.min_height   = min_height;
        cfg.mesh.classifier.max_height   = max_height;
        cfg.mesh.classifier.max_distance = max_distance;

        // Load the properties from the model
        auto loaded                     = load_model(path);
        class_map                       = loaded.class_map;
        cfg.model                       = loaded.model;
        cfg.mesh_model                  = loaded.mesh_model;
        cfg.num_classes                 = loaded.num_classes;
        cfg.mesh.geometry.shape         = loaded.geometry.shape;
        cfg.mesh.geometry.radius        = loaded.geometry.radius;
        cfg.mesh.geometry.intersections = loaded.geometry.intersections;

        runner = generate_runner::geometry(cfg);
    }

    VisualMeshResults VisualMeshRunner::operator()(const Image& image, const Eigen::Affine3f& Htc) {
        // Run our lambda
        return runner(image, Htc);
    }

}  // namespace module::vision::visualmesh
