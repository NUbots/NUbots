#ifndef MODULE_INPUT_FAKECAMERA_HPP
#define MODULE_INPUT_FAKECAMERA_HPP


#include <filesystem>
#include <memory>
#include <mutex>
#include <nuclear>
#include <string>
#include <turbojpeg.h>
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

        /// @brief List of images and their corresponding lens files that we are cycling through
        std::vector<std::pair<std::string, std::string>> images;
        /// @brief Index into the images vector representing the current image/lens pair we are loading
        size_t image_index = 0;
        /// @brief mutex controlling access to the images vector and the image_index variable
        std::mutex images_mutex;

        /// @brief JPEG decompressor. Constructed as a shared_ptr so that it will be automatically deleted on class
        /// destruction
        std::shared_ptr<void> decompressor = std::shared_ptr<void>(tjInitDecompress(), [](auto handle) {
            if (handle != nullptr) {
                tjDestroy(handle);
            }
        });

        /// @brief Read binary data from a file
        /// @param path Path to the file to read
        /// @return Vector of binary data
        std::vector<uint8_t> read_file(const std::filesystem::path& path);

    public:
        /// @brief Called by the powerplant to build and setup the FakeCamera reactor.
        explicit FakeCamera(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_FAKECAMERA_HPP
