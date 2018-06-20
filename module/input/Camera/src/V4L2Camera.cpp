#include "V4L2Camera.h"
#include "Camera.h"

#include <Eigen/Core>
#include <Eigen/LU>
#include <nuclear>

#include "extension/FileWatch.h"

#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"

namespace module {
namespace input {
    using extension::Configuration;
    using extension::FileWatch;

    using message::input::CameraParameters;

    using FOURCC = utility::vision::FOURCC;


    V4L2Camera Camera::initiateV4L2Camera(const Configuration& config) {
        // This trigger gets us as close as we can to the frame rate as possible (as high resolution as we can)
        V4L2FrameRateHandle =
            on<Every<V4L2Camera::FRAMERATE, Per<std::chrono::seconds>>, Single>().then("Read V4L2Camera", [this] {
                for (auto& camera : V4L2Cameras) {
                    try {
                        // If the camera is ready, get an image and emit it
                        if (camera.second.isStreaming()) {
                            emit<Scope::DIRECT>(std::make_unique<ImageData>(camera.second.getImage()));
                        }
                    }
                    catch (std::system_error& e) {
                        log<NUClear::ERROR>(e.what());
                        log("Resetting Camera");
                        V4L2FrameRateHandle.disable();
                        V4L2SettingsHandle.disable();
                        camera.second.resetCamera();
                        camera.second.startStreaming();
                        V4L2SettingsHandle.enable();
                        V4L2FrameRateHandle.enable();
                        log("Camera Reset");
                    }
                }
            });

        std::string deviceID = config["deviceID"];
        V4L2Camera camera    = V4L2Camera(config, deviceID, *this);

        V4L2SettingsHandle =
            on<Every<1, std::chrono::seconds>>().then("V4L2 Camera Setting Applicator", [this, config] {
                for (auto& camera : V4L2Cameras) {


                    try {
                        static uint errorCount = 0;

                        if (camera.second.isStreaming()) {
                            // Set all other camera settings
                            for (auto& setting : camera.second.getConfig().config) {

                                auto& settings = camera.second.getSettings();
                                auto it        = settings.find(setting.first.as<std::string>());

                                if (it != settings.end()) {
                                    if (camera.second.setSetting(it->second, setting.second.as<int>()) == false) {
                                        log<NUClear::DEBUG>("Failed to set", it->first, "on camera", camera.first);
                                        errorCount++;
                                    }
                                }
                            }
                            if (errorCount > config["max_error_count"].as<uint>()) {
                                errorCount = 0;
                                throw std::system_error(
                                    errno, std::system_category(), ("Camerea Settings Unresponsive"));
                            }
                        }
                    }
                    catch (std::system_error& e) {
                        log<NUClear::ERROR>(e.what());
                        log("Resetting Camera");
                        V4L2FrameRateHandle.disable();
                        V4L2SettingsHandle.disable();
                        camera.second.resetCamera();
                        camera.second.startStreaming();
                        V4L2SettingsHandle.enable();
                        V4L2FrameRateHandle.enable();
                        log("Camera Reset");
                    }
                }
            });

        camera.setSettingsHandle(V4L2SettingsHandle);
        auto cameraParameters = std::make_unique<CameraParameters>();
        double tanHalfFOV[2], imageCentre[2];

        // Generic camera parameters
        cameraParameters->imageSizePixels << config["imageWidth"].as<uint>(), config["imageHeight"].as<uint>();
        cameraParameters->FOV << config["FOV_X"].as<double>(), config["FOV_Y"].as<double>();
        // TODO: configure the offset? probably not necessary for pinhole
        cameraParameters->centreOffset = Eigen::Vector2i::Zero();

        // Pinhole specific
        cameraParameters->lens                     = CameraParameters::LensType::PINHOLE;
        tanHalfFOV[0]                              = std::tan(cameraParameters->FOV[0] * 0.5);
        tanHalfFOV[1]                              = std::tan(cameraParameters->FOV[1] * 0.5);
        imageCentre[0]                             = cameraParameters->imageSizePixels[0] * 0.5;
        imageCentre[1]                             = cameraParameters->imageSizePixels[1] * 0.5;
        cameraParameters->pinhole.distortionFactor = config["DISTORTION_FACTOR"].as<double>();
        cameraParameters->pinhole.pixelsToTanThetaFactor << (tanHalfFOV[0] / imageCentre[0]),
            (tanHalfFOV[1] / imageCentre[1]);
        cameraParameters->pinhole.focalLengthPixels = imageCentre[0] / tanHalfFOV[0];

        emit<Scope::DIRECT>(std::move(cameraParameters));

        log("Emitted pinhole camera parameters for camera", config["deviceID"].as<std::string>());

        try {
            // Recreate the camera device at the required resolution
            int width            = config["imageWidth"].as<uint>();
            int height           = config["imageHeight"].as<uint>();
            std::string deviceID = config["deviceID"].as<std::string>();
            std::string format   = config["imageFormat"].as<std::string>();
            FOURCC fourcc        = utility::vision::getFourCCFromDescription(format);

            log("Initialising driver for camera", deviceID);

            // V4L2Camera camera(config, deviceID);

            camera.resetCamera(deviceID, format, fourcc, width, height);

            log("Applying settings for camera", deviceID);

            // Set all other camera settings
            for (auto& setting : config.config) {
                auto& settings = camera.getSettings();
                auto it        = settings.find(setting.first.as<std::string>());

                if (it != settings.end()) {
                    if (camera.setSetting(it->second, setting.second.as<int>()) == false) {
                        throw std::system_error(
                            errno, std::system_category(), ("Failed to set", it->first, "on camera", deviceID));
                    }
                }
            }

            // Start the camera streaming video
            camera.startStreaming();

            log("Camera", deviceID, "is now streaming.");

            camera.enableHandles();

            return (std::move(camera));
        }

        catch (const std::exception& e) {
            NUClear::log<NUClear::DEBUG>(std::string("Exception while setting camera configuration: ") + e.what());
            throw e;
        }
    }

    ImageData V4L2Camera::getImage() {
        if (!streaming) {
            throw std::runtime_error("The camera is currently not streaming");
        }

        // Extract our buffer from the driver
        v4l2_buffer current;
        memset(&current, 0, sizeof(current));
        current.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        current.memory = V4L2_MEMORY_USERPTR;

        if (ioctl(fd, VIDIOC_DQBUF, &current) == -1) {
            throw std::system_error(errno, std::system_category(), "There was an error while de-queuing a buffer");
        }

        // Extract our data and create a new fresh buffer
        std::vector<uint8_t> data(buffers[current.index].size());
        std::swap(data, buffers[current.index]);
        data.resize(current.bytesused);

        // Calculate the timestamp in terms of NUClear clock
        auto monotonicTime =
            std::chrono::microseconds(current.timestamp.tv_usec) + std::chrono::seconds(current.timestamp.tv_sec);
        auto mclock =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch());
        auto nclock = std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch());
        auto timestamp = NUClear::clock::time_point(monotonicTime + (nclock - mclock));

        // Requeue our buffer
        v4l2_buffer requeue;
        memset(&requeue, 0, sizeof(requeue));
        requeue.type      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        requeue.memory    = V4L2_MEMORY_USERPTR;
        requeue.index     = current.index;
        requeue.m.userptr = reinterpret_cast<unsigned long int>(buffers[current.index].data());
        requeue.length    = buffers[current.index].capacity();

        if (ioctl(fd, VIDIOC_QBUF, &requeue) == -1) {
            throw std::system_error(errno, std::system_category(), "There was an error while re-queuing a buffer");
        }

        // Move this data into the image
        ImageData image;
        image.dimensions << width, height;
        image.format        = fourcc;
        image.serial_number = deviceID;
        image.timestamp     = timestamp;
        image.data          = std::move(data);
        image.isLeft        = false;  // Sorry future person... too lazy
        return image;
    }

    void V4L2Camera::resetCamera(const std::string& device,
                                 const std::string& fmt,
                                 const FOURCC& cc,
                                 size_t w,
                                 size_t h) {
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
        if (fd >= 0) {
            std::cout << "Reopened Camera" << std::endl;
        }

        // Check if we managed to open our file descriptor
        int resetCount = 0;

        while (fd < 0 && resetCount < 10) {
            std::cout << "Toggling GPIO" << std::endl;
            system("/bin/bash /home/nubots/gpio_toggle.sh");
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            fd = open(deviceID.c_str(), O_RDWR);
            resetCount++;
        }
        if (fd < 0) {
            throw std::runtime_error(std::string("We were unable to access the camera device on ") + deviceID);
        }

        // Here we set the "Format" of the device (the type of data we are getting)
        v4l2_format format;
        memset(&format, 0, sizeof(format));
        format.type           = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width  = width;
        format.fmt.pix.height = height;

        // We have to choose YUYV or MJPG here
        if (fmt == "YUYV") {
            format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        }

        else if (fmt == "MJPG") {
            format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        }

        else {
            throw std::runtime_error("The format must be either YUYV or MJPG");
        }

        format.fmt.pix.field = V4L2_FIELD_NONE;
        if (ioctl(fd, VIDIOC_S_FMT, &format) == -1) {
            throw std::system_error(
                errno, std::system_category(), "There was an error while setting the cameras format");
        }

        // Set the frame rate
        v4l2_streamparm param;
        memset(&param, 0, sizeof(param));
        param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        // Get the current parameters (populate our fields)
        if (ioctl(fd, VIDIOC_G_PARM, &param) == -1) {
            throw std::system_error(
                errno, std::system_category(), "We were unable to get the current camera FPS parameters");
        }

        param.parm.capture.timeperframe.numerator   = 1;
        param.parm.capture.timeperframe.denominator = FRAMERATE;

        if (ioctl(fd, VIDIOC_S_PARM, &param) == -1) {
            throw std::system_error(
                errno, std::system_category(), "We were unable to get the current camera FPS parameters");
        }

        // Tell V4L2 that we are using 2 userspace buffers
        v4l2_requestbuffers rb;
        memset(&rb, 0, sizeof(rb));
        rb.count  = NUM_BUFFERS;
        rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rb.memory = V4L2_MEMORY_USERPTR;

        if (ioctl(fd, VIDIOC_REQBUFS, &rb) == -1) {
            throw std::system_error(errno, std::system_category(), "There was an error configuring user buffers");
        }

        settings.insert(std::make_pair("brightness", V4L2_CID_BRIGHTNESS));
        settings.insert(std::make_pair("gain", V4L2_CID_GAIN));
        settings.insert(std::make_pair("gamma", V4L2_CID_GAMMA));
        settings.insert(std::make_pair("contrast", V4L2_CID_CONTRAST));
        settings.insert(std::make_pair("saturation", V4L2_CID_SATURATION));
        settings.insert(std::make_pair("power_line_frequency", V4L2_CID_POWER_LINE_FREQUENCY));
        settings.insert(std::make_pair("auto_white_balance", V4L2_CID_AUTO_WHITE_BALANCE));
        settings.insert(std::make_pair("white_balance_temperature", V4L2_CID_WHITE_BALANCE_TEMPERATURE));
        settings.insert(std::make_pair("auto_exposure", V4L2_CID_EXPOSURE_AUTO));
        settings.insert(std::make_pair("auto_exposure_priority", V4L2_CID_EXPOSURE_AUTO_PRIORITY));
        settings.insert(std::make_pair("absolute_exposure", V4L2_CID_EXPOSURE_ABSOLUTE));
        settings.insert(std::make_pair("backlight_compensation", V4L2_CID_BACKLIGHT_COMPENSATION));
        settings.insert(std::make_pair("auto_focus", V4L2_CID_FOCUS_AUTO));
        // settings.insert(std::make_pair("absolute_focus",             V4L2_CID_FOCUS_ABSOLUTE));
        settings.insert(std::make_pair("absolute_zoom", V4L2_CID_ZOOM_ABSOLUTE));
        settings.insert(std::make_pair("absolute_pan", V4L2_CID_PAN_ABSOLUTE));
        settings.insert(std::make_pair("absolute_tilt", V4L2_CID_TILT_ABSOLUTE));
        settings.insert(std::make_pair("sharpness", V4L2_CID_SHARPNESS));
    }

    void V4L2Camera::resetCamera() {
        resetCamera(deviceID, format, fourcc, width, height);
    }

    void V4L2Camera::startStreaming() {
        if (!streaming) {
            // Start streaming data
            int command = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (ioctl(fd, VIDIOC_STREAMON, &command) == -1) {
                throw std::system_error(errno, std::system_category(), "Unable to start camera streaming");
            }

            // Calculate how big our buffers must be
            size_t bufferlength = width * height * 2;

            // Enqueue 2 buffers
            for (uint i = 0; i < NUM_BUFFERS; ++i) {
                buffers[i].resize(bufferlength);

                v4l2_buffer buff;
                memset(&buff, 0, sizeof(buff));
                buff.type      = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buff.memory    = V4L2_MEMORY_USERPTR;
                buff.index     = i;
                buff.m.userptr = reinterpret_cast<unsigned long int>(buffers[i].data());
                buff.length    = buffers[i].capacity();

                if (ioctl(fd, VIDIOC_QBUF, &buff) == -1) {
                    throw std::system_error(errno, std::system_category(), "Unable to queue buffers");
                }
            }

            streaming = true;
        }
    }

    void V4L2Camera::stopStreaming() {
        if (streaming) {
            // Dequeue all buffers
            for (bool done = false; !done;) {
                if (ioctl(fd, VIDIOC_DQBUF) == -1) {
                    done = true;
                }
            }

            // Stop streaming data
            int command = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (ioctl(fd, VIDIOC_STREAMOFF, &command) == -1) {
                throw std::system_error(errno, std::system_category(), "Unable to stop camera streaming");
            }

            streaming = false;
        }
    }

    void V4L2Camera::closeCamera() {
        if (fd != -1) {
            try {
                stopStreaming();
            }
            catch (std::system_error) {
                streaming = false;
            }
            close(fd);
            fd = -1;
        }
    }

    void V4L2Camera::setConfig(const ::extension::Configuration& _config) {
        config = _config;

        int w           = config["imageWidth"].as<uint>();
        int h           = config["imageHeight"].as<uint>();
        std::string ID  = config["deviceID"].as<std::string>();
        std::string fmt = config["imageFormat"].as<std::string>();
        FOURCC cc       = utility::vision::getFourCCFromDescription(format);

        if (width != static_cast<size_t>(w) || height != static_cast<size_t>(h) || format != fmt || deviceID != ID) {
            resetCamera(ID, fmt, cc, w, h);
        }
    }

    int32_t V4L2Camera::getSetting(unsigned int id) {
        // Check if we can access the value
        struct v4l2_queryctrl queryctrl;
        queryctrl.id = id;

        if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) == -1) {
            throw std::system_error(errno,
                                    std::system_category(),
                                    "There was an error while attempting to get the status of this camera value");
        }

        if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
            throw std::runtime_error("Requested camera value is not available");
        }

        if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER
            && queryctrl.type != V4L2_CTRL_TYPE_MENU) {
            throw std::runtime_error("Requested camera value is not supported");
        }

        // Try to get the value
        struct v4l2_control control_s;
        control_s.id = id;

        if (ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0) {
            throw std::system_error(
                errno, std::system_category(), "There was an error while trying to get the current value");
        }

        return control_s.value;
    }

    bool V4L2Camera::setSetting(unsigned int id, int32_t value) {
        // Check if we can access the value
        struct v4l2_queryctrl queryctrl;
        queryctrl.id = id;

        if (ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
            return false;
        }

        if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
            return false;
        }

        if (queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER
            && queryctrl.type != V4L2_CTRL_TYPE_MENU) {
            return false;
        }

        // Shape the value if it's above or below our limits
        if (value < queryctrl.minimum) {
            value = queryctrl.minimum;
        }

        if (value > queryctrl.maximum) {
            value = queryctrl.maximum;
        }

        // Attempt to write the value
        struct v4l2_control control_s;
        control_s.id    = id;
        control_s.value = value;

        if (ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0) {
            return false;
        }

        // We succeeded
        return true;
    }
}  // namespace input
}  // namespace module
