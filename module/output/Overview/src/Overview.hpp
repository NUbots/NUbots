#ifndef MODULE_OUTPUT_OVERVIEW_HPP
#define MODULE_OUTPUT_OVERVIEW_HPP

#include <nuclear>

namespace module::output {

    class Overview : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the Overview reactor.
        explicit Overview(std::unique_ptr<NUClear::Environment> environment);

    private:
        NUClear::clock::time_point last_camera_image;
        NUClear::clock::time_point last_seen_ball;
        NUClear::clock::time_point last_seen_goal;
    };

}  // namespace module::output

#endif  // MODULE_OUTPUT_OVERVIEW_HPP
