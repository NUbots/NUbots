#ifndef MODULE_LOCALISATION_ODOMETRYLOCALISATION_H
#define MODULE_LOCALISATION_ODOMETRYLOCALISATION_H

#include <nuclear>

#include "utility/math/matrix/Transform2D.h"

namespace module {
namespace localisation {

    class OdometryLocalisation : public NUClear::Reactor {
    private:
        utility::math::matrix::Transform2D localisationOffset = {0, 0, 0};

    public:
        /// @brief Called by the powerplant to build and setup the OdometryLocalisation reactor.
        explicit OdometryLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };
}  // namespace localisation
}  // namespace module

#endif  // MODULE_LOCALISATION_ODOMETRYLOCALISATION_H
