/*
 * This file is part of DarwinCameraReader.
 *
 * DarwinCameraReader is free software: you can redistribute it and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * DarwinCameraReader is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with DarwinCameraReader.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 Michael Burton <michael.burton@uon.edu.au>, Jake Woods <jake.f.woods@gmail.com>,
 * Trent Houliston <trent@houliston.me>
 */

#ifndef MODULES_DARWINCAMERA_H
#define MODULES_DARWINCAMERA_H

#include <map>
#include <linux/videodev2.h>

#include "messages/Image.h"
#include "DarwinCameraSetting.h"

namespace modules {

    /**
     * @brief This class encapsulates the physical camera device. It will setup a camera device and begin streaming 
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
     */
    class DarwinCamera {
    private:
        /// @brief This struct holds all of the variables we need for interaction with the kernel space buffers
        struct KernelBuffers {
            /// @brief this is the v4l2 buffer object that we use to request data be sent to this buffer
            v4l2_buffer v4l2;
            
            /// @brief when using the kernel space buffers, they must be mapped into this programs virtual address space
            struct Data {
                /// @brief the size of the kernel space buffer
                size_t length;
                /// @brief a pointer to the first address of the virtual mapped kernel space
                void* payload;
            } data;
        } buff[2];

        /// @brief although this is a bool, it is used as an array index to swap between two buffers (0 and 1)
        bool activeBuffer;

        /// @brief this file descriptor points to the camera object
        int fd;
        
        /// @brief this map is used to hold several ioctl wrappers that let us set settings easily
        std::map<std::string, DarwinCameraSetting> settings;
    public:

        /// @brief this enum holds important constants (we are c++ we don't use defines for this kind of thing)
        enum {
            /// @brief the width of the image we are requesting
            WIDTH = 640,
            /// @brief the height of the image we are requesting
            HEIGHT = 480,
            /// @brief the total number of pixels we are requesting
            SIZE = WIDTH * HEIGHT * 2,
            /// @brief the framerate we are requesting
            FRAMERATE = 30
        };
        
        /**
         * @brief Constructs a new DarwinCamera class using the passed string as the device
         * 
         * @param device the path to the video device to use (e.g. /dev/video0)
         */
        DarwinCamera(const std::string& device);
        
        /**
         * @brief Gets a pointer to the latest image from the camera so that it can be sent out to the rest of the system
         * 
         * @return a pointer to the latest image from the camera
         */
        messages::Image* getImage();
        
        /**
         * @brief This method is to be called when shutting down the system. It does cleanup on the cameras resources
         */
        void closeCamera();
    } ;
}

#endif /* MODULES_DARWINCAMERA_H */

