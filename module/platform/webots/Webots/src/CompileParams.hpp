#ifndef MODULE_PLATFORM_WEBOTS_COMPILE_PARAMS
#define MODULE_PLATFORM_WEBOTS_COMPILE_PARAMS

#include <type_traits>

/**
 * These are things that we must know at compile time
 */

namespace module::platform::Webots {
    using simulation_tick_speed = std::integral_constant<int, 1>; // In milliseconds
}

#endif //MODULE_PLATFORM_WEBOTS_COMPILE_PARAMS
