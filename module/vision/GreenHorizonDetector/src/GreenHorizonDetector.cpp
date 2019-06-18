#include "GreenHorizonDetector.h"

#include <fmt/format.h>
#include <set>

#include "extension/Configuration.h"

#include "message/vision/GreenHorizon.h"
#include "message/vision/VisualMesh.h"

#include "utility/math/geometry/ConvexHull.h"
#include "utility/math/vision.h"

namespace module {
namespace vision {

    using extension::Configuration;

    using message::vision::VisualMesh;
    using GreenHorizonMsg = message::vision::GreenHorizon;

    static constexpr int LINE_INDEX  = 2;
    static constexpr int FIELD_INDEX = 3;

    GreenHorizonDetector::GreenHorizonDetector(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("GreenHorizonDetector.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file GreenHorizonDetector.yaml
            config.seed_confidence = cfg["seed_confidence"].as<float>();
            config.end_confidence  = cfg["end_confidence"].as<float>();
            config.cluster_points  = cfg["cluster_points"].as<int>();
            config.debug           = cfg["debug"].as<bool>();
        });

        on<Trigger<VisualMesh>, Buffer<2>>().then("Green Horizon", [this](const VisualMesh& mesh) {
            // Convinence variables
            const auto& cls        = mesh.classifications;
            const auto& neighbours = mesh.neighbourhood;
            const auto& coords     = mesh.coordinates;

            // Get some indices to partition
            std::vector<int> indices(mesh.indices.size());
            std::iota(indices.begin(), indices.end(), 0);

            // Partition the indices such that we only have the field points that dont have field surrounding them
            auto boundary = std::partition(indices.begin(), indices.end(), [&](const int& idx) {
                // If point is not a field or field line point then we also ignore it
                if ((cls(idx, FIELD_INDEX) + cls(idx, LINE_INDEX)) < config.end_confidence) {
                    return false;
                }
                // If at least one neighbour is not a field or a field line then this point should be on the edge
                for (int n = 4; n < 6; ++n) {
                    const int neighbour_idx = neighbours(idx, n);
                    if (neighbour_idx == indices.size()) {
                        continue;
                    }
                    if ((cls(neighbour_idx, FIELD_INDEX) + cls(neighbour_idx, LINE_INDEX)) < config.end_confidence) {
                        return true;
                    }
                }
                return false;
            });

            // Discard indices that are not on the boundary
            indices.resize(std::distance(indices.begin(), boundary));

            if (indices.size() < 3) {
                if (config.debug) {
                    log<NUClear::DEBUG>("Unable to make a convex hull with less than 3 points");
                }
            }
            else {
                // Find the convex hull of the cluster
                std::vector<int> horizon_indices(utility::math::geometry::upper_convex_hull(indices, coords));

                if (config.debug) {
                    log<NUClear::DEBUG>(fmt::format("Calculated convex hull with {} points from cluster with {} points",
                                                    horizon_indices.size(),
                                                    indices.size()));
                }

                auto msg = std::make_unique<GreenHorizonMsg>();

                // Preserve mesh so that anyone using the GreenHorizon can access the original data
                msg->mesh = const_cast<VisualMesh*>(&mesh)->shared_from_this();

                msg->camera_id = mesh.camera_id;
                msg->Hcw       = mesh.Hcw;

                // Convert indices to cam space unit vectors
                msg->horizon.reserve(horizon_indices.size());
                for (const auto& index : horizon_indices) {
                    auto unit = utility::math::vision::getCamFromImage(convert<float, 2>(coords.row(index)),
                                                                       convert<unsigned int, 2>(mesh.image->dimensions),
                                                                       mesh.image->lens);
                    msg->horizon.emplace_back(convert<double, 3>(unit).cast<float>());
                }

                emit(std::move(msg));
            }
        });
    }  // namespace vision
}  // namespace vision
}  // namespace module
