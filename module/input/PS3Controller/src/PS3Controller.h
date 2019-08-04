#ifndef MODULE_INPUT_PS3CONTROLLER_H
#define MODULE_INPUT_PS3CONTROLLER_H

#include <nuclear>
#include <string>
#include <vector>

namespace module {
namespace input {

    class PS3Controller : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the PS3Controller reactor.
        explicit PS3Controller(std::unique_ptr<NUClear::Environment> environment);

    private:
        void connect();
        void disconnect();

        int controller_fd;
        int accelerometer_fd;
        std::string controller_path;
        std::string accelerometer_path;
        ReactionHandle controller_reaction;
        ReactionHandle accelerometer_reaction;
    };

}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_PS3CONTROLLER_H
