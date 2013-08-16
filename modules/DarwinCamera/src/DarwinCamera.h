/*! @file DarwinCamera.h
    @brief Declaration of Darwin camera Reactor class

    @author Jason Kulk

  Copyright (c) 2009 Jason Kulk

 This file is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This file is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MODULES_DARWINCAMERA_H
#define MODULES_DARWINCAMERA_H

#include <NUClear.h>

// Forward declarations
struct v4l2_buffer;
namespace messages {
    class Image;
    class CameraSettings;
}

namespace modules {

    class DarwinCamera : public NUClear::Reactor {
    public:
        DarwinCamera(NUClear::PowerPlant& plant);
        ~DarwinCamera();

        messages::Image* grabNewImage();
        void applySettings(const messages::CameraSettings& newset);
        void forceApplySettings(const messages::CameraSettings& newset);

    private:
        enum {
            frameBufferCount = 1, //!< Number of available frame buffers.
            WIDTH = 640,
            HEIGHT = 480,
            SIZE = WIDTH * HEIGHT * 2,
            FRAMERATE = 30
        };

        bool applySetting(unsigned int settingID, int value);
        int readSetting(unsigned int id);
        void initialiseCamera();
        void readCameraSettings();
        void openCameraDevice(const std::string& device_name);
        void setStreaming(bool streaming_on);
        bool capturedNew();

        int fd;                                             //!< The file descriptor for the video device.
        void* mem[frameBufferCount];                        //!< Frame buffer addresses.
        int memLength[frameBufferCount];                   	//!< The length of each frame buffer.
        std::unique_ptr<struct v4l2_buffer> buf;            //!< Reusable parameter struct for some ioctl calls.
        bool bufQueued;                                     //!< Whether 'buf' points to a currently queued buffer
        std::unique_ptr<messages::CameraSettings> settings; //!< The current camera settings
    };
}
#endif

