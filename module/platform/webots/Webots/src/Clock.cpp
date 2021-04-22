#include "Clock.hpp"


namespace module::platform::webots {
    Clock::time_point Clock::now() {
        return time_point(current_tick);
    }

    void Clock::tick() {
        current_tick++;
    }

    Clock::duration Clock::current_tick = Clock::duration();
}  // namespace module::platform::webots
