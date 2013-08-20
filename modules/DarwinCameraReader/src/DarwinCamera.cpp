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

#include "DarwinCamera.h"

#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdexcept>
#include <system_error>
#include <sstream>

namespace modules {

    DarwinCamera::DarwinCamera(const std::string& device) : activeBuffer(false), fd(open(device.c_str(), O_RDWR)) {
        // Check if we managed to open our file descriptor
        if (fd < 0) {
            throw std::runtime_error(std::string("We were unable to access the camera device on ") + device);
        }

        // Do our configuration of the camera

        // Here we set the "Format" of the device (the type of data we are getting)
        v4l2_format format;
        memset(&format, 0, sizeof (format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = WIDTH;
        format.fmt.pix.height = HEIGHT;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field = V4L2_FIELD_NONE;
        if (ioctl(fd, VIDIOC_S_FMT, &format) == -1) {
            throw std::system_error(errno, std::system_category(), "There was an error while setting the cameras format");
        }

        if(format.fmt.pix.sizeimage != SIZE) {
            std::stringstream errorStream;
            errorStream 
                << "The camera returned an image size that made no sense (" 
                << "Expected: " << SIZE
                << ", "
                << "Found: " << format.fmt.pix.sizeimage 
                << ")";
            throw std::runtime_error(errorStream.str());
        }
        
        // Set the frame rate
        v4l2_streamparm param;
        memset(&param, 0, sizeof(param));
        param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // Get the current parameters (populate our fields)
        if(ioctl(fd, VIDIOC_G_PARM, &param) == -1) {
            throw std::system_error(errno, std::system_category(), "We were unable to get the current camera FPS parameters");
        }
        param.parm.capture.timeperframe.numerator = 1;
        param.parm.capture.timeperframe.denominator = FRAMERATE;
        if(ioctl(fd, VIDIOC_S_PARM, &param) == -1) {
            throw std::system_error(errno, std::system_category(), "We were unable to get the current camera FPS parameters");
        }
        
        // Request 2 kernel space buffers to read the data from the camera into
        struct v4l2_requestbuffers rb;
        memset(&rb, 0, sizeof(rb));
        rb.count = 2; // 2 buffers, one to queue one to read
        rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rb.memory = V4L2_MEMORY_MMAP;
        if(ioctl(fd, VIDIOC_REQBUFS, &rb) == -1) {
            throw std::system_error(errno, std::system_category(), "There was an error requesting the buffer");
        }
        
        // Map those two buffers into our user space so we can access them
        for(int i = 0; i < 2; ++i) {
            buff[i].v4l2.index = i;
            buff[i].v4l2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buff[i].v4l2.memory = V4L2_MEMORY_MMAP;
            if(ioctl(fd, VIDIOC_QUERYBUF, &buff[i]) == -1) {
                throw std::system_error(errno, std::system_category(), "There was an error mapping the video buffer into user space");
            }
            
            buff[i].data.length = buff[i].v4l2.length;
            buff[i].data.payload = mmap(0, buff[i].v4l2.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buff[i].v4l2.m.offset);
            
            if(buff[i].data.payload == MAP_FAILED) {
                throw std::runtime_error("There was an error mapping the video buffer into user space");
            }
        }

        // Enqueue our first buffer so that the kernel can write data to it
        if(ioctl(fd, VIDIOC_QBUF, &buff[0].v4l2) == -1) {
            throw std::system_error(errno, std::system_category(), "There was an error queuing buffers for the kernel to write to");
        }
        
        // Populate our settings table
        settings.insert(std::make_pair("autoWhiteBalance",        DarwinCameraSetting(fd, V4L2_CID_AUTO_WHITE_BALANCE)));
        settings.insert(std::make_pair("whiteBalanceTemperature", DarwinCameraSetting(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE)));
        settings.insert(std::make_pair("autoExposurePriority",    DarwinCameraSetting(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY)));
        settings.insert(std::make_pair("brightness",              DarwinCameraSetting(fd, V4L2_CID_BRIGHTNESS)));
        settings.insert(std::make_pair("contrast",                DarwinCameraSetting(fd, V4L2_CID_CONTRAST)));
        settings.insert(std::make_pair("saturation",              DarwinCameraSetting(fd, V4L2_CID_SATURATION)));
        settings.insert(std::make_pair("gain",                    DarwinCameraSetting(fd, V4L2_CID_GAIN)));
        settings.insert(std::make_pair("absoulteExposure",        DarwinCameraSetting(fd, V4L2_CID_EXPOSURE_ABSOLUTE)));
        settings.insert(std::make_pair("powerLineFrequency",      DarwinCameraSetting(fd, V4L2_CID_POWER_LINE_FREQUENCY)));
        settings.insert(std::make_pair("sharpness",               DarwinCameraSetting(fd, V4L2_CID_SHARPNESS)));
        
        // Start streaming data
        int command = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if(ioctl(fd, VIDIOC_STREAMON, &command) == -1) {
            throw std::system_error(errno, std::system_category(), "Unable to start camera streaming");
        }
    }

    messages::Image* DarwinCamera::getImage() {
        
        // Enqueue our next buffer so it can be written to
        if(ioctl(fd, VIDIOC_QBUF, &buff[!activeBuffer].v4l2) == -1) {
            throw std::system_error(errno, std::system_category(), "There was an error while re-queuing a buffer");
        }

        // Get our frame buffer with data in it
        if(ioctl(fd, VIDIOC_DQBUF, &buff[activeBuffer].v4l2) == -1) {
            throw std::system_error(errno, std::system_category(), "There was an error while de-queuing a buffer");
        }
        
        if(buff[activeBuffer].v4l2.bytesused != SIZE) {
            throw std::system_error(errno, std::system_category(), "A bad camera frame was returned (incorrect size)");
        }

        // Memcpy our data directly from the buffer
        std::unique_ptr<messages::Image::Pixel[]> data =
                std::unique_ptr<messages::Image::Pixel[]>(new messages::Image::Pixel[SIZE / 2]);
        // Create the image at quarter resolution
        messages::Image::Pixel* v4lbuff = static_cast<messages::Image::Pixel*>(buff[activeBuffer].data.payload);
        for(size_t i = 0; i < HEIGHT / 2; ++i) {
            std::copy(&v4lbuff[i * WIDTH], &v4lbuff[i * WIDTH + WIDTH / 2], &data.get()[i * WIDTH / 2]);
        }
        
        // Move this data into the image
        messages::Image* image = new messages::Image(WIDTH / 2, HEIGHT / 2, std::move(data));
        
        // Swap our buffers
        activeBuffer = !activeBuffer;
        
        // Return our image
        return image;
    }

    void DarwinCamera::closeCamera() {
        int command = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        // Start streaming data
        if(ioctl(fd, VIDIOC_STREAMOFF, &command) == -1) {
            throw std::system_error(errno, std::system_category(), "Unable to stop camera streaming");
        }
        
        close(fd);
    }
}
