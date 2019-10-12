#ifndef MODULE_TOOLS_PFTEST_H
#define MODULE_TOOLS_PFTEST_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <nuclear>
#include <utility>
#include <vector>

#include "VanDerPolModel.h"
#include "utility/math/filter/eigen/ParticleFilter.h"

namespace module {
namespace tools {

    class PFTest : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the UKFTest reactor.
        explicit PFTest(std::unique_ptr<NUClear::Environment> environment);

    private:
        double deltaT;

        NUClear::threading::ReactionHandle filter_handle;

        utility::math::filter::ParticleFilter<double, VanDerPolModel> model_filter;

        Eigen::Matrix<double, 1, 1> measurement_noise;

        std::vector<Eigen::Vector2d> true_state;
        std::vector<double> measurements;
        std::vector<double> innovations;
        std::vector<std::pair<utility::math::filter::ParticleFilter<double, VanDerPolModel>::StateVec,
                              utility::math::filter::ParticleFilter<double, VanDerPolModel>::StateMat>>
            actual_state;

        size_t time_count;
    };

}  // namespace tools
}  // namespace module

#endif  // MODULE_TOOLS_PFTEST_H
