#ifndef MODULE_LOCALISATION_ROBOTPARTICLELOCALISATION_H
#define MODULE_LOCALISATION_ROBOTPARTICLELOCALISATION_H

#include <nuclear>

#include "RobotModel.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Goal.h"
#include "utility/math/filter/ParticleFilter.h"

namespace module {
namespace localisation {

    class RobotParticleLocalisation : public NUClear::Reactor {
    private:
        utility::math::filter::ParticleFilter<RobotModel> filter;
        NUClear::clock::time_point last_time_update_time;
        NUClear::clock::time_point last_measurement_update_time;

        static constexpr int TIME_UPDATE_FREQUENCY     = 10;
        static constexpr int PARTICLE_UPDATE_FREQUENCY = 1;
        int draw_particles                             = 10;
        int n_particles;

        std::vector<arma::vec> getPossibleFieldPositions(const message::vision::Goal& goal,
                                                         const message::support::FieldDescription& fd) const;

    public:
        /// @brief Called by the powerplant to build and setup the RobotParticleLocalisation reactor.
        explicit RobotParticleLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace localisation
}  // namespace module

#endif  // MODULE_LOCALISATION_ROBOTPARTICLELOCALISATION_H
