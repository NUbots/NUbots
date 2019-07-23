#ifndef MODULE_PLATFORM_GAZEBO_H
#define MODULE_PLATFORM_GAZEBO_H

#include <nuclear>

namespace module {
namespace platform {

    class Gazebo : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Gazebo reactor.
        explicit Gazebo(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_PLATFORM_GAZEBO_H
