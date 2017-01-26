#ifndef MODULE_INPUT_CAMERA_H
#define MODULE_INPUT_CAMERA_H

#include <nuclear>

#include "extension/Configuration.h"

namespace module {
namespace input {

    class Camera : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Camera reactor.
        explicit Camera(std::unique_ptr<NUClear::Environment> environment);

    private:
    	void initiateV4L2Camera(const ::extension::Configuration& config);

    };

}
}

#endif  // MODULE_INPUT_CAMERA_H
