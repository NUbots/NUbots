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
                auto it = std::copy(payload, payload + current.bytesused, std::begin(jpegData));

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

            // TODO try adding these in on the new system
            // V4L2_CID_FOCUS_AUTO                 boolean     Enables automatic focus adjustments. The effect of manual focus adjustments while this feature is enabled is undefined, drivers should ignore such requests.
            // V4L2_CID_FOCUS_ABSOLUTE             integer     This control sets the focal point of the camera to the specified position. The unit is undefined. Positive values set the focus closer to the camera, negative values towards infinity.
            // V4L2_CID_BRIGHTNESS                 integer     Picture brightness, or more precisely, the black level.
            // V4L2_CID_CONTRAST                   integer     Picture contrast or luma gain.
            // V4L2_CID_SATURATION                 integer     Picture color saturation or chroma gain.
            // V4L2_CID_HUE                        integer     Hue or color balance.
            // V4L2_CID_EXPOSURE_ABSOLUTE          integer     Determines the exposure time of the camera sensor. The exposure time is limited by the frame interval. Drivers should interpret the values as 100 µs units, where the value 1 stands for 1/10000th of a second, 10000 for 1 second and 100000 for 10 seconds.
            // V4L2_CID_EXPOSURE_AUTO_PRIORITY     boolean     When V4L2_CID_EXPOSURE_AUTO is set to AUTO or APERTURE_PRIORITY, this control determines if the device may dynamically vary the frame rate. By default this feature is disabled (0) and the frame rate must remain constant.
            // V4L2_CID_AUTO_WHITE_BALANCE         boolean     Automatic white balance (cameras).
            // V4L2_CID_DO_WHITE_BALANCE           button      This is an action control. When set (the value is ignored), the device will do a white balance and then hold the current setting. Contrast this with the boolean V4L2_CID_AUTO_WHITE_BALANCE, which, when activated, keeps adjusting the white balance.
            // V4L2_CID_RED_BALANCE                integer     Red chroma balance.
            // V4L2_CID_BLUE_BALANCE               integer     Blue chroma balance.
            // V4L2_CID_GAMMA                      integer     Gamma adjust.
            // V4L2_CID_AUTOGAIN                   boolean     Automatic gain/exposure control.
            // V4L2_CID_GAIN                       integer     Gain control.
            // V4L2_CID_POWER_LINE_FREQUENCY       enum        Enables a power line frequency filter to avoid flicker. Possible values for enum v4l2_power_line_frequency are: V4L2_CID_POWER_LINE_FREQUENCY_DISABLED (0), V4L2_CID_POWER_LINE_FREQUENCY_50HZ (1) and V4L2_CID_POWER_LINE_FREQUENCY_60HZ (2)
            // V4L2_CID_HUE_AUTO                   boolean     Enables automatic hue control by the device. The effect of setting V4L2_CID_HUE while automatic hue control is enabled is undefined, drivers should ignore such request.
            // V4L2_CID_WHITE_BALANCE_TEMPERATURE  integer     This control specifies the white balance settings as a color temperature in Kelvin. A driver should have a minimum of 2800 (incandescent) to 6500 (daylight). For more information about color temperature see Wikipedia.
            // V4L2_CID_SHARPNESS                  integer     Adjusts the sharpness filters in a camera. The minimum value disables the filters, higher values give a sharper picture.
            // V4L2_CID_BACKLIGHT_COMPENSATION     integer     Adjusts the backlight compensation in a camera. The minimum value disables backlight compensation.
            // V4L2_CID_CHROMA_AGC                 boolean     Chroma automatic gain control.

            brightness                      (int): min=0 max=255 step=1 default=128 value=128
            contrast                        (int): min=0 max=255 step=1 default=128 value=128
            saturation                      (int): min=0 max=255 step=1 default=128 value=128
            white_balance_temperature_auto  (bool): default=1 value=1
            gain                            (int): min=0 max=255 step=1 default=0 value=0
            power_line_frequency            (menu): min=0 max=2 default=2 value=2
            white_balance_temperature       (int): min=2000 max=6500 step=1 default=4000 value=4000 flags=inactive
            sharpness                       (int): min=0 max=255 step=1 default=128 value=128
            backlight_compensation          (int): min=0 max=1 step=1 default=0 value=0
            exposure_auto                   (menu): min=0 max=3 default=3 value=3
            exposure_absolute               (int): min=3 max=2047 step=1 default=250 value=250 flags=inactive
            exposure_auto_priority          (bool)   : default=0 value=1
            pan_absolute                    (int): min=-36000 max=36000 step=3600 default=0 value=0
            tilt_absolute                   (int): min=-36000 max=36000 step=3600 default=0 value=0
            focus_absolute                  (int): min=0 max=250 step=5 default=0 value=0 flags=inactive
            focus_auto                      (bool): default=1 value=1
            zoom_absolute                   (int): min=100 max=500 step=1 default=100 value=100


            // Populate our settings table
            settings.insert(std::make_pair("autoWhiteBalance",        V4L2CameraSetting(fd, V4L2_CID_AUTO_WHITE_BALANCE)));
            settings.insert(std::make_pair("whiteBalanceTemperature", V4L2CameraSetting(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE)));
            settings.insert(std::make_pair("brightness",              V4L2CameraSetting(fd, V4L2_CID_BRIGHTNESS)));
            settings.insert(std::make_pair("contrast",                V4L2CameraSetting(fd, V4L2_CID_CONTRAST)));
            settings.insert(std::make_pair("saturation",              V4L2CameraSetting(fd, V4L2_CID_SATURATION)));
            settings.insert(std::make_pair("gain",                    V4L2CameraSetting(fd, V4L2_CID_GAIN)));
            settings.insert(std::make_pair("autoExposure",            V4L2CameraSetting(fd, V4L2_CID_EXPOSURE_AUTO))); // Can be set to V4L2_EXPOSURE_AUTO V4L2_EXPOSURE_MANUAL V4L2_EXPOSURE_SHUTTER_PRIORITY V4L2_EXPOSURE_APERTURE_PRIORITY
            settings.insert(std::make_pair("autoExposurePriority",    V4L2CameraSetting(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY)));
            settings.insert(std::make_pair("absoluteExposure",        V4L2CameraSetting(fd, V4L2_CID_EXPOSURE_ABSOLUTE)));
            settings.insert(std::make_pair("powerLineFrequency",      V4L2CameraSetting(fd, V4L2_CID_POWER_LINE_FREQUENCY)));
            settings.insert(std::make_pair("sharpness",               V4L2CameraSetting(fd, V4L2_CID_SHARPNESS)));
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