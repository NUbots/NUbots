#ifndef MODULE_LOCALISATION_ODOMETRYLOCALISATION_HPP
#define MODULE_LOCALISATION_ODOMETRYLOCALISATION_HPP

#include <Eigen/Geometry>
#include <nuclear>

namespace module::localisation {

    class OdometryLocalisation : public NUClear::Reactor {
    private:
        Eigen::Affine2d localisationOffset;

    public:
        /// @brief Called by the powerplant to build and setup the OdometryLocalisation reactor.
        explicit OdometryLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace module::localisation

#endif  // MODULE_LOCALISATION_ODOMETRYLOCALISATION_HPP
