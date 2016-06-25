#ifndef MODULE_LOCALISATION_ROBOTFIELDLOCALISATION_H
#define MODULE_LOCALISATION_ROBOTFIELDLOCALISATION_H

#include <nuclear>

namespace module {
namespace localisation {

    class RobotFieldLocalisation : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the RobotFieldLocalisation reactor.
        explicit RobotFieldLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_LOCALISATION_ROBOTFIELDLOCALISATION_H
