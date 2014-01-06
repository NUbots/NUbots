/*
 * This file is part of LinuxCamera.
 *
 * LinuxCamera is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LinuxCamera is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LinuxCamera.  If not, see <http://www.gnu.org/licenses/>.
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
        
        // For some reason MJPEGs dont have a huffman table
        constexpr uint8_t huffmantable[] = {
            0xC4, 0x01, 0xA2, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01,
            0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
            0x07, 0x08, 0x09, 0x0A, 0x0B, 0x01, 0x00, 0x03, 0x01,
            0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04,
            0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x10, 0x00,
            0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05,
            0x04, 0x04, 0x00, 0x00, 0x01, 0x7D, 0x01, 0x02, 0x03,
            0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
            0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81,
            0x91, 0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52,
            0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A,
            0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28,
            0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A,
            0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53,
            0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64,
            0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75,
            0x76, 0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86,
            0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96,
            0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6,
            0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
            0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6,
            0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6,
            0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5,
            0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4,
            0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0x11, 0x00, 0x02,
            0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04,
            0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03,
            0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51,
            0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14,
            0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52,
            0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34,
            0xE1, 0x25, 0xF1, 0x17, 0x18, 0x19, 0x1A, 0x26, 0x27,
            0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A,
            0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53,
            0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64,
            0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75,
            0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85,
            0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95,
            0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5,
            0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5,
            0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5,
            0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5,
            0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE2, 0xE3, 0xE4, 0xE5,
            0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5,
            0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xDA
        };
        
        V4L2Camera::V4L2Camera() : fd(-1), width(0), height(0), deviceID(""), streaming(false) , flipped(false){
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
                struct jpeg_decompress_struct cinfo = {0};
                
                uint8_t* payload = static_cast<uint8_t*>(buff[current.index].payload);

                // Create a decompressor
                jpeg_create_decompress(&cinfo);
                cinfo.err = jpeg_std_error(&err);
                
                // We need enough space for the table, and every byte except for byte 196
                std::vector<uint8_t> jpegData(current.bytesused + sizeof(huffmantable) - 1);
                
                // Copy our header (the first 195 bytes)
                auto it = std::copy(payload, payload + 195, std::begin(jpegData));
                
                // Copy our huffman table
                it = std::copy(std::begin(huffmantable), std::end(huffmantable), it);
                
                // Copy the remainder of our data
                std::copy(payload + 196, payload + current.bytesused, it);
                
                // Set our source buffer
                jpeg_mem_src(&cinfo, jpegData.data(), current.bytesused + sizeof(huffmantable) - 1);
                
                // Read our header
                jpeg_read_header(&cinfo, true);
                
                // Set our options
                cinfo.do_fancy_upsampling = false;
                cinfo.out_color_components = 3;
                cinfo.out_color_space = JCS_YCbCr;
                
                // Start decompression
                jpeg_start_decompress(&cinfo);

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
                image = std::unique_ptr<Image>(new Image(width, height, std::move(data), std::move(jpegData), flipped));
            }
            
            else {
                uint8_t* input = static_cast<uint8_t*>(buff[current.index].payload);
                
                const size_t total = width * height;
                
                // Fix the colour information to be YUV444 rather then YUV422
                for(size_t i = 0; i < total; ++++i) {
                    
                    data[total - i - 1].y  = input[i * 2];
                    data[total - i - 1].cb = input[i * 2 + 1];
                    data[total - i - 1].cr = input[i * 2 + 3];
                    
                    data[total - i].y  = input[i * 2 + 2];
                    data[total - i].cb = input[i * 2 + 1];
                    data[total - i].cr = input[i * 2 + 3];
                }                

                // Move this data into the image
                std::unique_ptr<Image> image = 
                        std::unique_ptr<Image>(new Image(width, height, std::move(data), flipped));
            }

            // Enqueue our next buffer so it can be written to
            if (ioctl(fd, VIDIOC_QBUF, &current) == -1) {
                throw std::system_error(errno, std::system_category(), "There was an error while re-queuing a buffer");
            }

            // Return our image
            return image;
        }

        void V4L2Camera::resetCamera(const std::string& device, const std::string& fmt, size_t w, size_t h, bool f) {
            // if the camera device is already open, close it
            closeCamera();

            // Store our new state
            deviceID = device;
            format = fmt;
            width = w;
            height = h;
            flipped = f;

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

            // Populate our settings table
            settings.insert(std::make_pair("autoWhiteBalance",        V4L2CameraSetting(fd, V4L2_CID_AUTO_WHITE_BALANCE)));
            settings.insert(std::make_pair("whiteBalanceTemperature", V4L2CameraSetting(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE)));
            settings.insert(std::make_pair("brightness",              V4L2CameraSetting(fd, V4L2_CID_BRIGHTNESS)));
            settings.insert(std::make_pair("contrast",                V4L2CameraSetting(fd, V4L2_CID_CONTRAST)));
            settings.insert(std::make_pair("saturation",              V4L2CameraSetting(fd, V4L2_CID_SATURATION)));
            settings.insert(std::make_pair("gain",                    V4L2CameraSetting(fd, V4L2_CID_GAIN)));
            settings.insert(std::make_pair("autoExposure",            V4L2CameraSetting(fd, V4L2_CID_EXPOSURE_AUTO)));
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