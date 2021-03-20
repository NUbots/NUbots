#ifndef MODULE_PLATFORM_WEBOTS_HPP
#define MODULE_PLATFORM_WEBOTS_HPP

#include <nuclear>

namespace module::platform {

class webots : public NUClear::Reactor {
private:
    /// @brief Send the inital message that tells webots who we are
    void send_connect(int fd, int teamId, int robotId);

public:
    /// @brief Called by the powerplant to build and setup the webots reactor.
    explicit webots(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::platform

#endif  // MODULE_PLATFORM_WEBOTS_HPP
