#ifndef MODULE_VISION_VISUALMESH_VISUALMESHRUNNER_HPP
#define MODULE_VISION_VISUALMESH_VISUALMESHRUNNER_HPP

#define CL_TARGET_OPENCL_VERSION 120
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "message/input/Image.hpp"

namespace module::vision::visualmesh {

    struct VisualMeshResults {
        Eigen::Matrix<float, 3, Eigen::Dynamic> rays;
        Eigen::Matrix<float, 2, Eigen::Dynamic> coordinates;
        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> neighbourhood;
        std::vector<int> indices;
        Eigen::MatrixXf classifications;
    };

    class VisualMeshRunner {
    private:
        int n_neighbours = 0;
        std::function<VisualMeshResults(const message::input::Image&, const Eigen::Isometry3f&)> runner;

    public:
        VisualMeshRunner(const std::string& engine,
                         const double& min_height,
                         const double& max_height,
                         const double& max_distance,
                         const double& intersection_tolerance,
                         const std::string& path,
                         const std::string& cache_directory);
        VisualMeshResults operator()(const message::input::Image& image, const Eigen::Isometry3f& Htc);

        /// Map of class names to class indices
        std::map<std::string, uint32_t> class_map;
    };

}  // namespace module::vision::visualmesh

#endif  // MODULE_VISION_VISUALMESH_VISUALMESHRUNNER_HPP
