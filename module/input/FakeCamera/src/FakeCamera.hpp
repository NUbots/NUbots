/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef MODULE_INPUT_FAKECAMERA_HPP
#define MODULE_INPUT_FAKECAMERA_HPP

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

    public:
        /// @brief Called by the powerplant to build and setup the FakeCamera reactor.
        explicit FakeCamera(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_FAKECAMERA_HPP
