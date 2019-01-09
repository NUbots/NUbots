#ifndef MODULE_MOTION_WALK_STATICWALK_H
#define MODULE_MOTION_WALK_STATICWALK_H

#include <nuclear>

namespace module {
namespace motion {
    namespace walk {

        class StaticWalk : public NUClear::Reactor {

        private:
            enum State { LEFT_LEAN, RIGHT_STEP, RIGHT_LEAN, LEFT_STEP } state;
            NUClear::clock::duration phase_time;
            NUClear::clock::time_point start_time;

        public:
            /// @brief Called by the powerplant to build and setup the StaticWalk reactor.
            explicit StaticWalk(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_STATICWALK_H
