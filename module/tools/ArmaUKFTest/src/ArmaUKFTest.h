#ifndef MODULE_TOOLS_ARMAUKFTEST_H
#define MODULE_TOOLS_ARMAUKFTEST_H

#include <armadillo>
#include <memory>
#include <nuclear>
#include <utility>
#include <vector>

#include "VanDerPolModel.h"
#include "utility/math/filter/UKF.h"

namespace module {
namespace tools {

    class ArmaUKFTest : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the UKFTest reactor.
        explicit ArmaUKFTest(std::unique_ptr<NUClear::Environment> environment);

    private:
        double deltaT;

        NUClear::threading::ReactionHandle filter_handle;

        utility::math::filter::UKF<module::tools::VanDerPolModel<double>> model_filter;

        double measurement_noise;

        std::vector<arma::vec2> true_state;
        std::vector<double> measurements;
        std::vector<double> innovations;
        std::vector<std::pair<utility::math::filter::UKF<module::tools::VanDerPolModel<double>>::StateVec,
                              utility::math::filter::UKF<module::tools::VanDerPolModel<double>>::StateMat>>
            actual_state;

        size_t time_count;
    };

}  // namespace tools
}  // namespace module

#endif  // MODULE_TOOLS_ARMAUKFTEST_H
