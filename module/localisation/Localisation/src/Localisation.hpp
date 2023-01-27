#ifndef MODULE_LOCALISATION_LOCALISATION_HPP
#define MODULE_LOCALISATION_LOCALISATION_HPP

#include <nuclear>

#include "message/input/Sensors.hpp"
#include "message/support/FieldDescription.hpp"
#include "message/support/nusight/DataPoint.hpp"
#include "message/vision/FieldLines.hpp"
#include "message/vision/Goal.hpp"

#include "utility/math/coordinates.hpp"
#include "utility/nusight/NUhelpers.hpp"

namespace module::localisation {

    class Localisation : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

        NUClear::clock::time_point last_time_update_time;
        NUClear::clock::time_point last_measurement_update_time;

        static constexpr int TIME_UPDATE_FREQUENCY = 10;

    public:
        /// @brief Called by the powerplant to build and setup the Localisation reactor.
        explicit Localisation(std::unique_ptr<NUClear::Environment> environment);
        Eigen::Vector3d ray2cartesian(Eigen::Vector3d ray, Eigen::Isometry3d Hcw);
        Eigen::Vector3d intersection(const Eigen::Vector3d& P0, const Eigen::Vector3d& V, double z);
    };


}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_LOCALISATION_HPP
