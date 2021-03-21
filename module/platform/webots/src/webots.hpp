#ifndef MODULE_PLATFORM_WEBOTS_HPP
#define MODULE_PLATFORM_WEBOTS_HPP

#include <nuclear>

namespace module::platform {

class Webots : public NUClear::Reactor {
private:
    /// @brief Send the inital message that tells webots who we are
    void send_connect(int fd, int team_id, int robot_id);

    /// @brief A single message that stores all the data that webots needs from us.
    ActuatorRequests to_send;

public:
    /// @brief Called by the powerplant to build and setup the webots reactor.
    explicit Webots(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::platform

#endif  // MODULE_PLATFORM_WEBOTS_HPP
