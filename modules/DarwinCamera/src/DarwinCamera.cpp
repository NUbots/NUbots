/*! @file DarwinCamera.cpp
    @brief Implementation of Darwin camera Reactor class

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

#include "DarwinCamera.h"
#include "messages/CameraSettings.h"
#include "messages/Image.h"

#include <cstring>
#include <sstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <cstdlib>
#include <unistd.h> /* Mitchell Metcalfe, 31-01-13, for GCC4.7, Ubuntu 12.10 compatibility */

#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#include <linux/version.h>
//#include <linux/i2c-dev.h>
#define __STRICT_ANSI__

#ifndef V4L2_CID_AUTOEXPOSURE
#  define V4L2_CID_AUTOEXPOSURE     (V4L2_CID_BASE+32)
#endif

#ifndef V4L2_CID_CAM_INIT
#  define V4L2_CID_CAM_INIT         (V4L2_CID_BASE+33)
#endif

#ifndef V4L2_CID_AUDIO_MUTE
#  define V4L2_CID_AUDIO_MUTE       (V4L2_CID_BASE+9)
#endif

namespace modules {

    DarwinCamera::DarwinCamera(NUClear::PowerPlant& plant): Reactor(plant), bufQueued(false) {
        on<Trigger<Every<1000 / FRAMERATE, std::chrono::milliseconds>>, Options<Single>>([this](const time_t& time) {
            emit(grabNewImage());
        });

        on<Trigger<Shutdown>>([this](const Shutdown& shutdown) {
            // disable streaming
            setStreaming(false);

            // unmap buffers
            for(int i = 0; i < frameBufferCount; ++i) {
                munmap(mem[i], memLength[i]);
            }

            // close the device
            close(fd);
        });

        on<Trigger<messages::CameraSettings>>([this](const messages::CameraSettings& settings) {
            applySettings(settings);
        });

        settings.reset(new messages::CameraSettings);

        // Open device
        openCameraDevice("/dev/video0");

        // Initialise
        initialiseCamera();
        readCameraSettings();

        // enable streaming
        setStreaming(true);
    }

    DarwinCamera::~DarwinCamera() {
    }

    void DarwinCamera::openCameraDevice(const std::string& device_name) {
        // open device
        fd = open(device_name.c_str(), O_RDWR);
        if(fd == -1) {
            throw std::runtime_error("Unable to open camera device");
        }
    }

    void DarwinCamera::setStreaming(bool streaming_on) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        int instruction = streaming_on ? VIDIOC_STREAMON: VIDIOC_STREAMOFF;
        if(ioctl(fd, instruction, &type) == -1) {
            throw std::runtime_error(streaming_on ? "Unable to start camera streaming" : "Unable to stop camera streaming");
        }
    }

    void DarwinCamera::initialiseCamera() {
        // set default parameters
        /*
        struct v4l2_control control;
        memset(&control, 0, sizeof(control));
        control.id = V4L2_CID_CAM_INIT;
        control.value = 0;
        if(ioctl(fd, VIDIOC_S_CTRL, &control) < 0) {
            throw std::runtime_error("DarwinCamera::initialiseCamera(): V4L2_CID_CAM_INIT failed");
        }
        
        v4l2_std_id esid0 = (WIDTH == 320 ? 0x04000000UL : 0x08000000UL);
        if(ioctl(fd, VIDIOC_S_STD, &esid0)) {
            throw std::runtime_error("DarwinCamera::initialiseCamera(): Error setting video mode");
        }
        */

        // set format
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = WIDTH;
        fmt.fmt.pix.height = HEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if(ioctl(fd, VIDIOC_S_FMT, &fmt)) {
            throw std::runtime_error("DarwinCamera::initialiseCamera(): error setting format");
        }

        if(fmt.fmt.pix.sizeimage != SIZE) {
            throw std::runtime_error("DarwinCamera::initialiseCamera(): image size incorrect");
        }

        // set frame rate
        struct v4l2_streamparm fps;
        memset(&fps, 0, sizeof(fps));
        fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if(ioctl(fd, VIDIOC_G_PARM, &fps)) {
            throw std::runtime_error("DarwinCamera::initialiseCamera(): error setting FPS");
        }
        fps.parm.capture.timeperframe.numerator = 1;
        fps.parm.capture.timeperframe.denominator = FRAMERATE;
        if(ioctl(fd, VIDIOC_S_PARM, &fps) == -1) {
            throw std::runtime_error("DarwinCamera::initialiseCamera(): error setting FPS");
        }

        // request buffers
        struct v4l2_requestbuffers rb;
        memset(&rb, 0, sizeof(rb));
        rb.count = frameBufferCount;
        rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rb.memory = V4L2_MEMORY_MMAP;
        if(ioctl(fd, VIDIOC_REQBUFS, &rb) == -1) {
            throw std::runtime_error("DarwinCamera::initialiseCamera(): error requesting buffers");
        }

        // map the buffers
        buf.reset(new v4l2_buffer);
        for(int i = 0; i < frameBufferCount; ++i) {
            buf->index = i;
            buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf->memory = V4L2_MEMORY_MMAP;
            if(ioctl(fd, VIDIOC_QUERYBUF, buf.get()) == -1) {
                throw std::runtime_error("DarwinCamera::initialiseCamera(): error mapping buffers");
            }
            memLength[i] = buf->length;
            mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
            if(mem[i] == MAP_FAILED) {
                throw std::runtime_error("DarwinCamera::initialiseCamera(): error mapping buffers");
            }
        }

        // queue the buffers
        for(int i = 0; i < frameBufferCount; ++i) {
            buf->index = i;
            buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf->memory = V4L2_MEMORY_MMAP;
            if(ioctl(fd, VIDIOC_QBUF, buf.get()) == -1) {
                throw std::runtime_error("DarwinCamera::initialiseCamera(): error queueing buffers");
            }
        }
    }

    bool DarwinCamera::capturedNew() {
        // requeue the buffer of the last captured image which is obselete now
        auto start = std::chrono::high_resolution_clock::now();
        if(bufQueued && ioctl(fd, VIDIOC_QBUF, buf.get()) == -1) {
            throw std::runtime_error("DarwinCamera::captureNew(): error re-queueing buffer");
        }

        // dequeue a frame buffer (this call blocks when there is no new image available) */
        if(ioctl(fd, VIDIOC_DQBUF, buf.get()) == -1) {
            throw std::runtime_error("DarwinCamera::captureNew(): error de-queueing buffer");
        }
        bufQueued = true;
        std::cout << "Dequeueing buffer took " << std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start).count() << "ms." << std::endl;

        if(buf->bytesused != SIZE) {
            throw std::runtime_error("DarwinCamera::captureNew(): image size incorrect");
        }

        return true;
    }

    messages::Image* DarwinCamera::grabNewImage() {
        while(!capturedNew());
        messages::Image* image = new messages::Image;
        image->copyFromYUV422Buffer(static_cast<messages::Image::Pixel*>(mem[buf->index]), WIDTH, HEIGHT, true);
        return image;
    }

    void DarwinCamera::readCameraSettings() {
        settings->exposureAuto               = readSetting(V4L2_CID_EXPOSURE_AUTO);
        settings->autoWhiteBalance           = readSetting(V4L2_CID_AUTO_WHITE_BALANCE);
        settings->whiteBalanceTemperature    = readSetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE);
        settings->exposureAutoPriority       = readSetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY);
        settings->brightness                 = readSetting(V4L2_CID_BRIGHTNESS);
        settings->contrast                   = readSetting(V4L2_CID_CONTRAST);
        settings->saturation                 = readSetting(V4L2_CID_SATURATION);
        settings->gain                       = readSetting(V4L2_CID_GAIN);
        settings->exposureAbsolute           = readSetting(V4L2_CID_EXPOSURE_ABSOLUTE);
        settings->powerLineFrequency         = readSetting(V4L2_CID_POWER_LINE_FREQUENCY);
        settings->sharpness                  = readSetting(V4L2_CID_SHARPNESS);

        settings->p_exposureAuto.set(settings->exposureAuto);
        settings->p_autoWhiteBalance.set(settings->autoWhiteBalance);
        settings->p_whiteBalanceTemperature.set(settings->whiteBalanceTemperature);
        settings->p_exposureAutoPriority.set(settings->exposureAutoPriority);
        settings->p_brightness.set(settings->brightness);
        settings->p_contrast.set(settings->contrast);
        settings->p_saturation.set(settings->saturation);
        settings->p_gain.set(settings->gain);
        settings->p_exposureAbsolute.set(settings->exposureAbsolute);
        settings->p_powerLineFrequency.set(settings->powerLineFrequency);
        settings->p_sharpness.set(settings->sharpness);
    }

    int DarwinCamera::readSetting(unsigned int id) {
        struct v4l2_queryctrl queryctrl;
        queryctrl.id = id;
        if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
            return -1;
        }
        if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
            return -1; // not available
        }
        if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU) {
            return -1; // not supported
        }

        struct v4l2_control control_s;
        control_s.id = id;
        if(ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0) {
            return -1;
        }
        if(control_s.value == queryctrl.default_value) {
            return -1;
        }
        return control_s.value;
    }

    bool DarwinCamera::applySetting(unsigned int id, int value) {
        struct v4l2_queryctrl queryctrl;
        queryctrl.id = id;
        if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
            return false;
        }
        if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
            return false; // not available
        }
        if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU) {
            return false; // not supported
        }
        // clip value
        if(value < queryctrl.minimum) {
            value = queryctrl.minimum;
        }
        if(value > queryctrl.maximum) {
            value = queryctrl.maximum;
        }

        struct v4l2_control control_s;
        control_s.id = id;
        control_s.value = value;
        if(ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0) {
            return false;
        }
        return true;
    }

    void DarwinCamera::applySettings(const messages::CameraSettings& newset) {
        // AutoSettings
        if(newset.p_exposureAuto.get() != settings->p_exposureAuto.get()) {
            settings->p_exposureAuto.set(newset.p_exposureAuto.get());
            applySetting(V4L2_CID_EXPOSURE_AUTO, (settings->p_exposureAuto.get()));
        }
        if(newset.p_autoWhiteBalance.get() != settings->p_autoWhiteBalance.get()) {
            settings->p_autoWhiteBalance.set(newset.p_autoWhiteBalance.get());
            applySetting(V4L2_CID_AUTO_WHITE_BALANCE, (settings->p_autoWhiteBalance.get()));
        }
        if(newset.p_whiteBalanceTemperature.get() != settings->p_whiteBalanceTemperature.get()) {
            settings->p_whiteBalanceTemperature.set(newset.p_whiteBalanceTemperature.get());
            applySetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, (settings->p_whiteBalanceTemperature.get()));
        }
        if(newset.p_exposureAutoPriority.get() != settings->p_exposureAutoPriority.get()) {
            settings->p_exposureAutoPriority.set(newset.p_exposureAutoPriority.get());
            applySetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY, (settings->p_exposureAutoPriority.get()));
        }

        //Other controls
        if(newset.p_brightness.get() != settings->p_brightness.get()) {
            settings->p_brightness.set(newset.p_brightness.get());
            applySetting(V4L2_CID_BRIGHTNESS, (settings->p_brightness.get()));
        }
        if(newset.p_contrast.get() != settings->p_contrast.get()) {
            settings->p_contrast.set(newset.p_contrast.get());
            applySetting(V4L2_CID_CONTRAST, (settings->p_contrast.get()));
        }
        if(newset.p_saturation.get() != settings->p_saturation.get()) {
            settings->p_saturation.set(newset.p_saturation.get());
            applySetting(V4L2_CID_SATURATION, (settings->p_saturation.get()));
        }
        if(newset.p_gain.get() != settings->p_gain.get()) {
            settings->p_gain.set(newset.p_gain.get());
            applySetting(V4L2_CID_GAIN, (settings->p_gain.get()));
        }
        if(newset.p_exposureAbsolute.get() != settings->p_exposureAbsolute.get()) {
            settings->p_exposureAbsolute.set(newset.p_exposureAbsolute.get());
            applySetting(V4L2_CID_EXPOSURE_ABSOLUTE, (settings->p_exposureAbsolute.get()));
        }
        if(newset.p_powerLineFrequency.get() != settings->p_powerLineFrequency.get()) {
            settings->p_powerLineFrequency.set(newset.p_powerLineFrequency.get());
            applySetting(V4L2_CID_POWER_LINE_FREQUENCY, (settings->p_powerLineFrequency.get()));
        }
        if(newset.p_sharpness.get() != settings->p_sharpness.get()) {
            settings->p_sharpness.set(newset.p_sharpness.get());
            applySetting(V4L2_CID_SHARPNESS, (settings->p_sharpness.get()));
        }

        //COPIES INTO OLD FORMAT:
        settings->copyParams();
    }

    void DarwinCamera::forceApplySettings(const messages::CameraSettings& newset) {
        //Copying the new Paramters into m_settings
        *settings = newset;

        // Auto Controls
        applySetting(V4L2_CID_EXPOSURE_AUTO, (settings->p_exposureAuto.get()));
        applySetting(V4L2_CID_AUTO_WHITE_BALANCE, (settings->p_autoWhiteBalance.get()));
        applySetting(V4L2_CID_WHITE_BALANCE_TEMPERATURE, (settings->p_whiteBalanceTemperature.get()));
        applySetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY, (settings->p_exposureAutoPriority.get()));
        //Other controls
        applySetting(V4L2_CID_BRIGHTNESS, (settings->p_brightness.get()));
        applySetting(V4L2_CID_CONTRAST, (settings->p_contrast.get()));
        applySetting(V4L2_CID_SATURATION, (settings->p_saturation.get()));
        applySetting(V4L2_CID_GAIN, (settings->p_gain.get()));
        applySetting(V4L2_CID_EXPOSURE_ABSOLUTE, (settings->p_exposureAbsolute.get()));
        applySetting(V4L2_CID_POWER_LINE_FREQUENCY, (settings->p_powerLineFrequency.get()));
        applySetting(V4L2_CID_SHARPNESS, (settings->p_sharpness.get()));

        //COPIES INTO OLD FORMAT:
        settings->copyParams();
    }

}
