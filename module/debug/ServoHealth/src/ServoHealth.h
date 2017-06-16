#ifndef MODULE_DEBUG_SERVOHEALTH_H
#define MODULE_DEBUG_SERVOHEALTH_H

#include <nuclear>

#include "utility/input/ServoID.h"
#include "message/support/ServoHealthTestData.h"

namespace module {
namespace debug {

    class ServoHealth : public NUClear::Reactor {
    private:
        static constexpr size_t N_BUCKETS = 1024;

        const size_t id;
        message::support::ServoHealthTestData::State state;
        double fallen_angle;
        int counter = 0;
        int test_loops = 5;

    public: 
        /// @brief Called by the powerplant to build and setup the ServoHealth reactor.
        explicit ServoHealth(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_DEBUG_SERVOHEALTH_H
