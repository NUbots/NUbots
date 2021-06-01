#ifndef MODULE_INPUT_FAKECAMERA_HPP
#define MODULE_INPUT_FAKECAMERA_HPP

#include <mutex>
#include <nuclear>
#include <string>
#include <vector>

namespace module::input {

    class FakeCamera : public NUClear::Reactor {
    private:
        /// The configuration variables for this reactor
        struct {
            std::string image_folder;
            std::string image_prefix;
            std::string lens_prefix;
        } config;

        std::vector<std::pair<std::string, std::string>> images;
        size_t image_counter = 0;
        std::mutex images_mutex;

    public:
        /// @brief Called by the powerplant to build and setup the FakeCamera reactor.
        explicit FakeCamera(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_FAKECAMERA_HPP
