/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULES_INPUT_V4L2CAMERA_H
#define MODULES_INPUT_V4L2CAMERA_H

#include <map>
#include <memory>
#include <nuclear>
#include <string>

#include "extension/Configuration.h"

#include "ImageData.h"

#include "utility/vision/Vision.h"

namespace module {
namespace input {

    using FOURCC = utility::vision::FOURCC;

    constexpr size_t NUM_BUFFERS = 2;

    /**
     * @brief This struct encapsulates the physical camera device. It will setup a camera device and begin streaming
     *    images
     *
     * @details
     *    This class uses the Video4Linux2 kernel drivers in order to connect to and get data from the darwins built in
     *    webcam. It allocates 2 kernel mode buffers which are mapped into user space. These buffers are alternated and
     *  the data from the most recently filled one is used to construct an image object. This class also provides easy
     *  access to all the settings that are available on the camera. It is provided as a map in order to make accessing
     *  the paramters by name (from a config file) easier
     *
     * @author Michael Burton
     * @author Jake Woods
     * @author Trent Houliston
     * @author Matthew Amos
     */
    struct V4L2Camera {
    private:
        /// @brief Our two arrays of data that will be populated
        std::array<std::vector<uint8_t>, NUM_BUFFERS> buffers;

        /// @brief the width of the image being retrieved from the camera
        size_t width;

        /// @brief the height of the image being retrieved from the camera
        size_t height;

        /// @brief this map is used to hold several ioctl wrappers that let us set settings easily
        std::map<std::string, unsigned int> settings;

        /// @brief The name of the device to read camera data from
        std::string deviceID;

        /// @brief the format that we are reading in from the camera
        std::string format;
        FOURCC fourcc;

        /// @brief Whether the camera is currently in streaming mode
        bool streaming;

        NUClear::threading::ReactionHandle cameraHandle;
        NUClear::threading::ReactionHandle settingsHandle;

        /// @brief Configuration information for this camera.
        ::extension::Configuration config;

        NUClear::Reactor& reactor;

    public:
        /// @brief this file descriptor points to the camera object
        int fd;

        /// @brief this enum holds important constants (we are c++ we don't use defines for this kind of thing)
        enum {
            /// @brief the framerate we are requesting
            FRAMERATE = 30
        };

        /**
         * @brief Constructs a new DarwinCamera class using the passed string as the device
         *
         * @param device the path to the video device to use (e.g. /dev/video0)
         */
        V4L2Camera(const ::extension::Configuration& config, const std::string& deviceID, NUClear::Reactor& reactor)
            : buffers()
            , width(0)
            , height(0)
            , settings()
            , deviceID(deviceID)
            , format("")
            , fourcc(FOURCC::UNKNOWN)
            , streaming(false)
            , cameraHandle()
            , settingsHandle()
            , config(config)
            , reactor(reactor)
            , fd(-1) {}

        /**
         * @brief Gets a pointer to the latest image from the camera so that it can be sent out to the rest of the
         * system
         *
         * @details
         *   This function blocks until the camera device provides a new frame of video
         *   data, at which point it copies the frame into a new Image and returns. The
         *   camera device must already be set up (using resetCamera) and
         *
         * @return a pointer to the latest image from the camera
         */
        ImageData getImage();

        /**
         * @brief Sets up the camera at a given resolution
         *
         * @param name the name of the camera device
         * @param w the image's width
         * @param h the image's height
         * @param f whether the camera is mounted upside down
         */
        void resetCamera(const std::string& device, const std::string& fmt, const FOURCC& cc, size_t w, size_t h);

        /**
         * @brief Sets up the camera with previously set values
         */
        void resetCamera();

        /**
         * @brief Returns a map of all configurable settings
         */
        std::map<std::string, unsigned int>& getSettings() {
            return settings;
        }

        NUClear::threading::ReactionHandle& setCameraHandle() {
            return cameraHandle;
        }

        void unbindCameraHandle() {
            cameraHandle.disable();
            cameraHandle.unbind();
        }

        void enableHandles() {
            settingsHandle.enable();
            cameraHandle.enable();
        }

        void disableHandles() {
            settingsHandle.disable();
            cameraHandle.disable();
        }

        NUClear::threading::ReactionHandle& getSettingsHandle() {
            return settingsHandle;
        }

        void setCameraHandle(NUClear::threading::ReactionHandle camH) {
            cameraHandle = camH;
        }

        void setSettingsHandle(NUClear::threading::ReactionHandle setH) {
            settingsHandle = setH;
        }

        /**
         * @brief Returns the horizontal resolution the camera is currently set to
         */
        size_t getWidth() const {
            return width;
        }

        /**
         * @brief Returns the vertical resolution the camera is currently set to
         */
        size_t getHeight() const {
            return height;
        }

        /**
         * @brief Returns the device id that is currently used as the camera
         */
        const std::string& getDeviceID() const {
            return deviceID;
        }

        /**
         * @brief returns the format that the camera is currently reading (YUYV or MJPG)
         */
        const std::string& getFormat() const {
            return format;
        }

        int getFile() {
            return fd;
        }

        /**
         * @brief This method is to be called when shutting down the system. It does cleanup on the cameras resources
         */
        void closeCamera();

        /**
         * @brief Starts the camera streaming video
         */
        void startStreaming();

        /**
         * @brief Stops the camera streaming video
         */
        void stopStreaming();

        /**
         * @brief Check whether the camera is actively streaming video
         */
        bool isStreaming() const {
            return streaming;
        }

        /**
         * @brief Return configuration information for this camera.
         */
        const ::extension::Configuration& getConfig() const {
            return config;
        }

        /**
         * @brief Set configuration information for this camera.
         */
        void setConfig(const ::extension::Configuration& _config);


        int32_t getSetting(unsigned int id);

        bool setSetting(unsigned int id, int32_t value);
    };

}  // namespace input
}  // namespace module


#endif  // MODULES_INPUT_V4L2CAMERA_H
