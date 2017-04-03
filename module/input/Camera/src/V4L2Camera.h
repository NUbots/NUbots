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

#ifndef MODULES_INPUT_V4L2CAMERA_H
#define MODULES_INPUT_V4L2CAMERA_H

#include <map>
#include <memory>
#include <string>

#include "extension/Configuration.h"

#include "message/input/Image.h"

#include "utility/vision/fourcc.h"

namespace module 
{
    namespace input 
    {
        using message::input::Image;

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
         */
        struct V4L2Camera 
        {
        private:
            /// @brief Our two arrays of data that will be populated
            std::array<std::vector<uint8_t>, NUM_BUFFERS> buffers;

            /// @brief this file descriptor points to the camera object
            int fd;

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

            /// @brief The ID of this camera.
            int cameraID;

            /// @brief Configuration information for this camera.
            ::extension::Configuration config;

        public:

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
            V4L2Camera(const ::extension::Configuration& config, const std::string& deviceID, int cameraID) 
                : buffers()
                , fd(-1)
                , width(0)
                , height(0)
                , settings()
                , deviceID(deviceID)
                , format("")
                , fourcc(FOURCC::UNKNOWN)
                , streaming(false)
                , cameraID(cameraID)
                , config(config) {}

            /**
             * @brief Gets a pointer to the latest image from the camera so that it can be sent out to the rest of the system
             *
             * @details
             *   This function blocks until the camera device provides a new frame of video
             *   data, at which point it copies the frame into a new Image and returns. The
             *   camera device must already be set up (using resetCamera) and
             *
             * @return a pointer to the latest image from the camera
             */
            message::input::Image getImage() 
            {
                if (!streaming) 
                {
                    throw std::runtime_error("The camera is currently not streaming");
                }

                // Extract our buffer from the driver
                v4l2_buffer current;
                memset(&current, 0, sizeof(current));
                current.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                current.memory = V4L2_MEMORY_USERPTR;

                if (ioctl(fd, VIDIOC_DQBUF, &current) == -1)
                {
                    throw std::system_error(errno, std::system_category(), "There was an error while de-queuing a buffer");
                }

                // Extract our data and create a new fresh buffer
                std::vector<uint8_t> data(buffers[current.index].size());
                std::swap(data, buffers[current.index]);
                data.resize(current.bytesused);

                // Calculate the timestamp in terms of NUClear clock
                auto monotonicTime = std::chrono::microseconds(current.timestamp.tv_usec) + std::chrono::seconds(current.timestamp.tv_sec);
                auto mclock = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
                auto nclock = std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch());
                auto timestamp = NUClear::clock::time_point(monotonicTime + (nclock - mclock));

                // Requeue our buffer
                v4l2_buffer requeue;
                memset(&requeue, 0, sizeof(requeue));
                requeue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                requeue.memory = V4L2_MEMORY_USERPTR;
                requeue.index = current.index;
                requeue.m.userptr = reinterpret_cast<unsigned long int>(buffers[current.index].data());
                requeue.length = buffers[current.index].capacity();

                if (ioctl(fd, VIDIOC_QBUF, &requeue) == -1)
                {
                    throw std::system_error(errno, std::system_category(), "There was an error while re-queuing a buffer");
                };

                // Move this data into the image
                Image image;
                image.dimensions   << width, height;
                image.format       = fourcc;
                image.serialNumber = deviceID;
                image.timestamp    = timestamp;
                image.data         = std::move(data);
                image.camera_id    = cameraID;
                return image;
            }

            /**
             * @brief Sets up the camera at a given resolution
             *
             * @param name the name of the camera device
             * @param w the image's width
             * @param h the image's height
             * @param f whether the camera is mounted upside down
             */
            bool resetCamera(const std::string& device, const std::string& fmt, const FOURCC& cc, size_t w, size_t h)
            {
                // if the camera device is already open, close it
                closeCamera();

                // Store our new state
                deviceID = device;
                format   = fmt;
                fourcc   = cc;
                width    = w;
                height   = h;

                // Open the camera device
                fd = open(deviceID.c_str(), O_RDWR);

                // Check if we managed to open our file descriptor
                uint8_t resetCount = 0;

                while (fd < 0 && resetCount < 5)
                {   
                   
                    
                    std::ofstream gpio;
                    gpio.open ("/sys/class/gpio/gpio8/value");
                    gpio << "0";
                    std::cout << "off" << "\n\r";
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    gpio << "1";
                    std::cout << "on" << "\n\r";
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    gpio.close();
                    resetCount++;
                    fd = open(deviceID.c_str(), O_RDWR);
                    std::cout << "open" << "\n\r";

                    if (fd < 0) {
                        return false;
                         throw std::runtime_error(std::string("We were unable to access the camera device on ") + deviceID);
                     }
                }

                // Here we set the "Format" of the device (the type of data we are getting)
                v4l2_format format;
                memset(&format, 0, sizeof (format));
                format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                format.fmt.pix.width = width;
                format.fmt.pix.height = height;

                // We have to choose YUYV or MJPG here
                if(fmt == "YUYV")
                {
                    format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                }

                else if(fmt == "MJPG") 
                {
                    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
                }

                else
                {
                    throw std::runtime_error("The format must be either YUYV or MJPG");
                }

                format.fmt.pix.field = V4L2_FIELD_NONE;
                if (ioctl(fd, VIDIOC_S_FMT, &format) == -1)
                {
                    throw std::system_error(errno, std::system_category(), "There was an error while setting the cameras format");
                }

                // Set the frame rate
                v4l2_streamparm param;
                memset(&param, 0, sizeof(param));
                param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                // Get the current parameters (populate our fields)
                if (ioctl(fd, VIDIOC_G_PARM, &param) == -1)
                {
                    throw std::system_error(errno, std::system_category(), "We were unable to get the current camera FPS parameters");
                }

                param.parm.capture.timeperframe.numerator = 1;
                param.parm.capture.timeperframe.denominator = FRAMERATE;

                if (ioctl(fd, VIDIOC_S_PARM, &param) == -1)
                {
                    throw std::system_error(errno, std::system_category(), "We were unable to get the current camera FPS parameters");
                }

                // Tell V4L2 that we are using 2 userspace buffers
                v4l2_requestbuffers rb;
                memset(&rb, 0, sizeof(rb));
                rb.count = NUM_BUFFERS;
                rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                rb.memory = V4L2_MEMORY_USERPTR;

                if (ioctl(fd, VIDIOC_REQBUFS, &rb) == -1)
                {
                    throw std::system_error(errno, std::system_category(), "There was an error configuring user buffers");
                }

                settings.insert(std::make_pair("brightness",                 V4L2_CID_BRIGHTNESS));
                settings.insert(std::make_pair("gain",                       V4L2_CID_GAIN));
                settings.insert(std::make_pair("gamma",                      V4L2_CID_GAMMA));
                settings.insert(std::make_pair("contrast",                   V4L2_CID_CONTRAST));
                settings.insert(std::make_pair("saturation",                 V4L2_CID_SATURATION));
                settings.insert(std::make_pair("power_line_frequency",       V4L2_CID_POWER_LINE_FREQUENCY));
                settings.insert(std::make_pair("auto_white_balance",         V4L2_CID_AUTO_WHITE_BALANCE));
                settings.insert(std::make_pair("white_balance_temperature",  V4L2_CID_WHITE_BALANCE_TEMPERATURE));
                settings.insert(std::make_pair("auto_exposure",              V4L2_CID_EXPOSURE_AUTO));
                settings.insert(std::make_pair("auto_exposure_priority",     V4L2_CID_EXPOSURE_AUTO_PRIORITY));
                settings.insert(std::make_pair("absolute_exposure",          V4L2_CID_EXPOSURE_ABSOLUTE));
                settings.insert(std::make_pair("backlight_compensation",     V4L2_CID_BACKLIGHT_COMPENSATION));
                settings.insert(std::make_pair("auto_focus",                 V4L2_CID_FOCUS_AUTO));
                // settings.insert(std::make_pair("absolute_focus",             V4L2_CID_FOCUS_ABSOLUTE));
                settings.insert(std::make_pair("absolute_zoom",              V4L2_CID_ZOOM_ABSOLUTE));
                settings.insert(std::make_pair("absolute_pan",               V4L2_CID_PAN_ABSOLUTE));
                settings.insert(std::make_pair("absolute_tilt",              V4L2_CID_TILT_ABSOLUTE));
                settings.insert(std::make_pair("sharpness",                  V4L2_CID_SHARPNESS));
                return true;
            }

            /**
             * @brief Returns a map of all configurable settings
             */
            std::map<std::string, unsigned int>& getSettings() { return settings; }

            /**
             * @brief Returns the horizontal resolution the camera is currently set to
             */
            size_t getWidth() const { return width; }

            /**
             * @brief Returns the vertical resolution the camera is currently set to
             */
            size_t getHeight() const { return height; }

            /**
             * @brief Returns the device id that is currently used as the camera
             */
            const std::string& getDeviceID() const { return deviceID; }

            /**
             * @brief returns the format that the camera is currently reading (YUYV or MJPG)
             */
            const std::string& getFormat() const { return format; }

            /**
             * @brief This method is to be called when shutting down the system. It does cleanup on the cameras resources
             */
            void closeCamera()
            {
                if (fd != -1)
                {
                    stopStreaming();

                    close(fd);
                    fd = -1;
                }                
            }

            /**
             * @brief Starts the camera streaming video
             */
            void startStreaming()
            {
               if (!streaming)
               {
                    // Start streaming data
                    int command = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                    if (ioctl(fd, VIDIOC_STREAMON, &command) == -1)
                    {
                        throw std::system_error(errno, std::system_category(), "Unable to start camera streaming");
                    }

                    // Calculate how big our buffers must be
                    size_t bufferlength = width * height * 2;

                    // Enqueue 2 buffers
                    for(uint i = 0; i < NUM_BUFFERS; ++i)
                    {
                        buffers[i].resize(bufferlength);

                        v4l2_buffer buff;
                        memset(&buff, 0, sizeof(buff));
                        buff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buff.memory = V4L2_MEMORY_USERPTR;
                        buff.index = i;
                        buff.m.userptr = reinterpret_cast<unsigned long int>(buffers[i].data());
                        buff.length = buffers[i].capacity();

                        if (ioctl(fd, VIDIOC_QBUF, &buff) == -1)
                        {
                            throw std::system_error(errno, std::system_category(), "Unable to queue buffers");
                        }
                    }

                    streaming = true;
                }                
            }

            /**
             * @brief Check whether the camera is actively streaming video
             */
            bool isStreaming() const { return streaming; }

            /**
             * @brief Return configuration information for this camera.
             */
            const ::extension::Configuration& getConfig() const { return config; }

            /**
             * @brief Set configuration information for this camera.
             */
            void setConfig(const ::extension::Configuration& _config)
            {
                config = _config;

                int w  = config["imageWidth"].as<uint>();
                int h = config["imageHeight"].as<uint>();
                std::string ID = config["deviceID"].as<std::string>();
                std::string fmt   = config["imageFormat"].as<std::string>();
                FOURCC cc = utility::vision::getFourCCFromDescription(format);

                if (width    != static_cast<size_t>(w) ||
                    height   != static_cast<size_t>(h) ||
                    format   != fmt ||
                    deviceID != ID)
                {
                    resetCamera(ID, fmt, cc, w, h);
                }                
            }

            /**
             * @brief Stops the camera streaming video
             */
            void stopStreaming()
            {
                if (streaming)
                {
                    // Dequeue all buffers
                    for(bool done = false; !done;)
                    {
                        if(ioctl(fd, VIDIOC_DQBUF) == -1)
                        {
                            done = true;
                        }
                    }

                    // Stop streaming data
                    int command = V4L2_BUF_TYPE_VIDEO_CAPTURE;

                    if (ioctl(fd, VIDIOC_STREAMOFF, &command) == -1)
                    {
                        throw std::system_error(errno, std::system_category(), "Unable to stop camera streaming");
                    }

                    streaming = false;
                }                
            }

            int32_t getSetting(unsigned int id)
            {
                // Check if we can access the value
                struct v4l2_queryctrl queryctrl;
                queryctrl.id = id;

                if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == -1) 
                {
                    throw std::system_error(errno, std::system_category(), "There was an error while attempting to get the status of this camera value");
                }

                if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
                {
                    throw std::runtime_error("Requested camera value is not available");
                }

                if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
                {
                    throw std::runtime_error("Requested camera value is not supported");
                }

                // Try to get the value
                struct v4l2_control control_s;
                control_s.id = id;

                if (ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0)
                {
                    throw std::system_error(errno, std::system_category(), "There was an error while trying to get the current value");
                }

                return control_s.value;
            }

            bool setSetting(unsigned int id, int32_t value)
            {
                // Check if we can access the value
                struct v4l2_queryctrl queryctrl;
                queryctrl.id = id;

                if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
                {
                    return false;
                }

                if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
                {
                    return false;
                }

                if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
                {
                    return false;
                }

                // Shape the value if it's above or below our limits
                if (value < queryctrl.minimum)
                {
                    value = queryctrl.minimum;
                }

                if (value > queryctrl.maximum)
                {
                    value = queryctrl.maximum;
                }

                // Attempt to write the value
                struct v4l2_control control_s;
                control_s.id = id;
                control_s.value = value;

                if (ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0)
                {
                    return false;
                }

                // We succeeded
                return true;
            }
        };

    }  // input
}  // modules

#endif  // MODULES_INPUT_V4L2CAMERA_H

