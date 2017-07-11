#ifndef MODULE_BEHAVIOUR_TOOLS_SERVOTEST_H
#define MODULE_BEHAVIOUR_TOOLS_SERVOTEST_H

#include <nuclear>

namespace module {
namespace behaviour {
    namespace tools {

        class ServoTest : public NUClear::Reactor {

        public:
            /// @brief Called by the powerplant to build and setup the ServoTest reactor.
            explicit ServoTest(std::unique_ptr<NUClear::Environment> environment);
        };
    }  // namespace tools
}  // namespace behaviour
}  // namespace module

#endif  // MODULE_BEHAVIOUR_TOOLS_SERVOTEST_H
