#ifndef MODULE_SUPPORT_VORONOI_HPP
#define MODULE_SUPPORT_VORONOI_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "utility/voronoi/Voronoi.hpp"

namespace module::support {

    class Voronoi : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
            size_t robots_each_side = 0;
        } cfg;

        // utility::voronoi::Voronoi vd;

    public:
        /// @brief Called by the powerplant to build and setup the Voronoi reactor.
        explicit Voronoi(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::support

#endif  // MODULE_SUPPORT_VORONOI_HPP
