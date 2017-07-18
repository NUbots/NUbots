#ifndef MODULE_VISION_OBSTACLEDETECTOR_H
#define MODULE_VISION_OBSTACLEDETECTOR_H

#include <nuclear>

#include "utility/learning/KMeans.h"

namespace module {
namespace vision {

    class ObstacleDetector : public NUClear::Reactor {

        utility::learning::KMeans kmeansClusterer;

    public:
        /// @brief Called by the powerplant to build and setup the ObstacleDetector reactor.
        explicit ObstacleDetector(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace vision
}  // namespace module

#endif  // MODULE_VISION_OBSTACLEDETECTOR_H
