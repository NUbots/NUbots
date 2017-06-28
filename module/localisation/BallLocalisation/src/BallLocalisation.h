#ifndef MODULE_LOCALISATION_BALLLOCALISATION_H
#define MODULE_LOCALISATION_BALLLOCALISATION_H

#include <nuclear>

#include "utility/math/filter/UKF.h"
#include "utility/math/filter/ParticleFilter.h"
#include "BallModel.h"

namespace module {
namespace localisation {

    class BallLocalisation : public NUClear::Reactor {
    private:
        // utility::math::filter::UKF<BallModel> filter;
        utility::math::filter::ParticleFilter<BallModel> filter;
        NUClear::clock::time_point last_time_update_time;
        NUClear::clock::time_point last_measurement_update_time;
    public:
        /// @brief Called by the powerplant to build and setup the BallLocalisation reactor.
        explicit BallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_H
