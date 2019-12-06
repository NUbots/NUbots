#ifndef MODULE_LOCALISATION_ODOMETRYLOCALISATION_H
#define MODULE_LOCALISATION_ODOMETRYLOCALISATION_H

#include <Eigen/Geometry>
#include <nuclear>

namespace module {
namespace localisation {

    class OdometryLocalisation : public NUClear::Reactor {
    private:
        Eigen::Affine2d localisationOffset = Eigen::Affine2d::Identity();

    public:
        /// @brief Called by the powerplant to build and setup the OdometryLocalisation reactor.
        explicit OdometryLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace localisation
}  // namespace module

#endif  // MODULE_LOCALISATION_ODOMETRYLOCALISATION_H
