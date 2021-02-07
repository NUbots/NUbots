#ifndef MODULE_LOCALISATION_ODOMETRYLOCALISATION_HPP
#define MODULE_LOCALISATION_ODOMETRYLOCALISATION_HPP

#include <nuclear>

#include "utility/math/matrix/Transform2D.hpp"

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

#endif  // MODULE_LOCALISATION_ODOMETRYLOCALISATION_HPP
