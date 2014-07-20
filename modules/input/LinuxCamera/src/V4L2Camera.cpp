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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "V4L2Camera.h"

#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdexcept>
#include <system_error>
#include <string>
#include <sstream>
#include <linux/videodev2.h>
#include <jpeglib.h>

namespace modules {
    namespace input {

        using messages::input::Image;

        V4L2Camera::V4L2Camera() : fd(-1), width(0), height(0), deviceID(""), streaming(false) {
        }

        std::unique_ptr<Image> V4L2Camera::getImage() {
            if (!streaming) {
                return nullptr;
            }

            v4l2_buffer current;
            memset(&current, 0, sizeof(current));
            current.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            current.memory = V4L2_MEMORY_MMAP;

            // Get our frame buffer with data in it
            if (ioctl(fd, VIDIOC_DQBUF, &current) == -1) {
                throw std::system_error(errno, std::system_category(), "There was an error while de-queuing a buffer");
            }

            std::vector<Image::Pixel> data(width * height);
            std::unique_ptr<Image> image;

            // If it is a MJPG
            if(format == "MJPG") {
                struct jpeg_error_mgr err;
                struct jpeg_decompress_struct cinfo;
                std::memset(&cinfo, 0, sizeof(jpeg_decompress_struct));

                uint8_t* payload = static_cast<uint8_t*>(buff[current.index].payload);

                // Create a decompressor
                jpeg_create_decompress(&cinfo);
                cinfo.err = jpeg_std_error(&err);

                std::vector<uint8_t> jpegData(current.bytesused);

                // Copy our header (the first 195 bytes)
                std::copy(payload, payload + current.bytesused, std::begin(jpegData));

                // Set our source buffer
                jpeg_mem_src(&cinfo, jpegData.data(), current.bytesused);

                // Read our header
                jpeg_read_header(&cinfo, true);

                // Set our options
                cinfo.do_fancy_upsampling = false;
                cinfo.out_color_components = 3;
                cinfo.out_color_space = JCS_YCbCr;

                // Start decompression
                jpeg_start_decompress(&cinfo);

                // Decompress the JPEG
                for (Image::Pixel* row = data.data();
                        cinfo.output_scanline < cinfo.output_height;
                        row += width) {

                    // Read the scanline into place
                    jpeg_read_scanlines(&cinfo, reinterpret_cast<uint8_t**>(&row), 1);
                }

                // Clean up
                jpeg_finish_decompress(&cinfo);
                jpeg_destroy_decompress(&cinfo);

                // Move this data into the image along with the jpeg source
                image = std::unique_ptr<Image>(new Image(width, height, std::move(data), std::move(jpegData)));
            }

            else {
                uint8_t* input = static_cast<uint8_t*>(buff[current.index].payload);

                const size_t total = width * height;

                // Fix the colour information to be YUV444 rather then YUV422
                for(size_t i = 0; i < total; ++++i) {

                    data[i].y  = input[i * 2];
                    data[i].cb = input[i * 2 + 1];
                    data[i].cr = input[i * 2 + 3];

                    data[i + 1].y  = input[i * 2 + 2];
                    data[i + 1].cb = input[i * 2 + 1];
                    data[i + 1].cr = input[i * 2 + 3];
                }

                // Move this data into the image
                image = std::unique_ptr<Image>(new Image(width, height, std::move(data)));
            }

            // Enqueue our next buffer so it can be written to
            if (ioctl(fd, VIDIOC_QBUF, &current) == -1) {
                throw std::system_error(errno, std::system_category(), "There was an error while re-queuing a buffer");
            }

            // Return our image
            return image;
        }

        void V4L2Camera::resetCamera(const std::string& device, const std::string& fmt, size_t w, size_t h) {
            // if the camera device is already open, close it
            closeCamera();

            // Store our new state
            deviceID = device;
            format = fmt;
            width = w;
            height = h;

            // Open the camera device
            fd = open(deviceID.c_str(), O_RDWR);
            // Check if we managed to open our file descriptor
            if (fd < 0) {
                throw std::runtime_error(std::string("We were unable to access the camera device on ") + deviceID);
            }

            // Here we set the "Format" of the device (the type of data we are getting)
            v4l2_format format;
            memset(&format, 0, sizeof (format));
            format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            format.fmt.pix.width = width;
            format.fmt.pix.height = height;

            // We have to choose YUYV or MJPG here
            if(fmt == "YUYV") {
                format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            }
            else if(fmt == "MJPG") {
                format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
            }
            else {
                throw std::runtime_error("The format must be either YUYV or MJPG");
            }

            format.fmt.pix.field = V4L2_FIELD_NONE;
            if (ioctl(fd, VIDIOC_S_FMT, &format) == -1) {
                throw std::system_error(errno, std::system_category(), "There was an error while setting the cameras format");
            }

            // Set the frame rate
            v4l2_streamparm param;
            memset(&param, 0, sizeof(param));
            param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            // Get the current parameters (populate our fields)
            if (ioctl(fd, VIDIOC_G_PARM, &param) == -1) {
                throw std::system_error(errno, std::system_category(), "We were unable to get the current camera FPS parameters");
            }
            param.parm.capture.timeperframe.numerator = 1;
            param.parm.capture.timeperframe.denominator = FRAMERATE;
            if (ioctl(fd, VIDIOC_S_PARM, &param) == -1) {
                throw std::system_error(errno, std::system_category(), "We were unable to get the current camera FPS parameters");
            }

            // Request 2 kernel space buffers to read the data from the camera into
            v4l2_requestbuffers rb;
            memset(&rb, 0, sizeof(rb));
            rb.count = 2; // 2 buffers, one to queue one to read
            rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            rb.memory = V4L2_MEMORY_MMAP;
            if (ioctl(fd, VIDIOC_REQBUFS, &rb) == -1) {
                throw std::system_error(errno, std::system_category(), "There was an error requesting the buffer");
            }

            // Map those two buffers into our user space so we can access them
            for (int i = 0; i < 2; ++i) {
                v4l2_buffer buffer;
                buffer.index = i;
                buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buffer.memory = V4L2_MEMORY_MMAP;
                if (ioctl(fd, VIDIOC_QUERYBUF, &buffer) == -1) {
                    throw std::system_error(errno, std::system_category(), "There was an error mapping the video buffer into user space");
                }

                buff[i].length = buffer.length;
                buff[i].payload = mmap(0, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buffer.m.offset);

                if (buff[i].payload == MAP_FAILED) {
                    throw std::runtime_error("There was an error mapping the video buffer into user space");
                }

                // Enqueue our buffer so that the kernel can write data to it
                if (ioctl(fd, VIDIOC_QBUF, &buffer) == -1) {
                    throw std::system_error(errno, std::system_category(), "There was an error queuing buffers for the kernel to write to");
                }
            }

            settings.insert(std::make_pair("brightness",                 V4L2CameraSetting(fd, V4L2_CID_BRIGHTNESS)));
            settings.insert(std::make_pair("gain",                       V4L2CameraSetting(fd, V4L2_CID_GAIN)));
            settings.insert(std::make_pair("contrast",                   V4L2CameraSetting(fd, V4L2_CID_CONTRAST)));
            settings.insert(std::make_pair("saturation",                 V4L2CameraSetting(fd, V4L2_CID_SATURATION)));
            settings.insert(std::make_pair("power_line_frequency",        V4L2CameraSetting(fd, V4L2_CID_POWER_LINE_FREQUENCY)));
            settings.insert(std::make_pair("auto_white_balance",         V4L2CameraSetting(fd, V4L2_CID_AUTO_WHITE_BALANCE)));
            settings.insert(std::make_pair("white_balance_temperature",  V4L2CameraSetting(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE)));
            settings.insert(std::make_pair("auto_exposure",              V4L2CameraSetting(fd, V4L2_CID_EXPOSURE_AUTO)));
            settings.insert(std::make_pair("auto_exposure_priority",     V4L2CameraSetting(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY)));
            settings.insert(std::make_pair("absolute_exposure",          V4L2CameraSetting(fd, V4L2_CID_EXPOSURE_ABSOLUTE)));
            settings.insert(std::make_pair("backlight_compensation",     V4L2CameraSetting(fd, V4L2_CID_BACKLIGHT_COMPENSATION)));
            settings.insert(std::make_pair("auto_focus",                 V4L2CameraSetting(fd, V4L2_CID_FOCUS_AUTO)));
            settings.insert(std::make_pair("absolute_focus",             V4L2CameraSetting(fd, V4L2_CID_FOCUS_ABSOLUTE)));
            settings.insert(std::make_pair("absolute_zoom",              V4L2CameraSetting(fd, V4L2_CID_ZOOM_ABSOLUTE)));
            settings.insert(std::make_pair("absolute_pan",               V4L2CameraSetting(fd, V4L2_CID_PAN_ABSOLUTE)));
            settings.insert(std::make_pair("absolute_tilt",              V4L2CameraSetting(fd, V4L2_CID_TILT_ABSOLUTE)));
            settings.insert(std::make_pair("sharpness",                  V4L2CameraSetting(fd, V4L2_CID_SHARPNESS)));
        }

        void V4L2Camera::startStreaming() {
            if (!streaming) {
                // Start streaming data
                int command = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (ioctl(fd, VIDIOC_STREAMON, &command) == -1) {
                    throw std::system_error(errno, std::system_category(), "Unable to start camera streaming");
                }

                streaming = true;
            }
        }

        void V4L2Camera::stopStreaming() {
            if (streaming) {
                int command = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                // Stop streaming data
                if (ioctl(fd, VIDIOC_STREAMOFF, &command) == -1) {
                    throw std::system_error(errno, std::system_category(), "Unable to stop camera streaming");
                }

                streaming = false;
            }
        }

        bool V4L2Camera::isStreaming() const {
            return streaming;
        }

        std::map<std::string, V4L2CameraSetting>& V4L2Camera::getSettings() {
            return settings;
        }

        size_t V4L2Camera::getWidth() const {
            return width;
        }

        size_t V4L2Camera::getHeight() const {
            return height;
        }

        const std::string& V4L2Camera::getDeviceID() const {
            return deviceID;
        }

        const std::string& V4L2Camera::getFormat() const {
            return format;
        }

        void V4L2Camera::closeCamera() {
            if (fd != -1) {
                stopStreaming();

                // unmap buffers
                for (int i = 0; i < 2; ++i) {
                    munmap(buff[i].payload, buff[i].length);
                }

                close(fd);
                fd = -1;
            }
        }

    }  // input
}  // modules