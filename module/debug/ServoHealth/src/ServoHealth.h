#ifndef MODULE_DEBUG_SERVOHEALTH_H
#define MODULE_DEBUG_SERVOHEALTH_H

#include <nuclear>

#include "utility/input/ServoID.h"

namespace module {
namespace debug {

    class ServoHealth : public NUClear::Reactor {
    private:
    	const size_t id;
    	std::map<utility::input::ServoID, std::array<int, 4096>> loadHealth;

    public: 
        /// @brief Called by the powerplant to build and setup the ServoHealth reactor.
        explicit ServoHealth(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_DEBUG_SERVOHEALTH_H
