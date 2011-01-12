/*! @file NAOCamera.cpp
    @brief Implementation of NAO camera class

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

#include "NAOCamera.h"
#include "NUPlatform/NUPlatform.h"
#include "GTAssert.h"

#include "debug.h"
#include "debugverbositynucamera.h"
#include "nubotdataconfig.h"        // for initial camera settings location

#include <cstring>
#include <sstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdlib.h>

#undef __STRICT_ANSI__
#include <linux/videodev.h>
#include <linux/version.h>
#include <bn/i2c/i2c-dev.h>
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

/*
#ifndef V4L2_CID_POWER_LINE_FREQUENCY
#  define V4L2_CID_POWER_LINE_FREQUENCY  (V4L2_CID_BASE+24)
enum v4l2_power_line_frequency {
  V4L2_CID_POWER_LINE_FREQUENCY_DISABLED  = 0,
  V4L2_CID_POWER_LINE_FREQUENCY_50HZ  = 1,
  V4L2_CID_POWER_LINE_FREQUENCY_60HZ  = 2,
};

#define V4L2_CID_HUE_AUTO      (V4L2_CID_BASE+25)
#define V4L2_CID_WHITE_BALANCE_TEMPERATURE  (V4L2_CID_BASE+26)
	#define V4L2_CID_SHARPNESS      (V4L2_CID_BASE+27)
#define V4L2_CID_BACKLIGHT_COMPENSATION   (V4L2_CID_BASE+28)

#define V4L2_CID_CAMERA_CLASS_BASE     (V4L2_CTRL_CLASS_CAMERA | 0x900)
#define V4L2_CID_CAMERA_CLASS       (V4L2_CTRL_CLASS_CAMERA | 1)

#define V4L2_CID_EXPOSURE_AUTO      (V4L2_CID_CAMERA_CLASS_BASE+1)
enum  v4l2_exposure_auto_type {
  V4L2_EXPOSURE_MANUAL = 0,
  V4L2_EXPOSURE_AUTO = 1,
  V4L2_EXPOSURE_SHUTTER_PRIORITY = 2,
  V4L2_EXPOSURE_APERTURE_PRIORITY = 3
};
#define V4L2_CID_EXPOSURE_ABSOLUTE    (V4L2_CID_CAMERA_CLASS_BASE+2)
#define V4L2_CID_EXPOSURE_AUTO_PRIORITY    (V4L2_CID_CAMERA_CLASS_BASE+3)

#define V4L2_CID_PAN_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+4)
#define V4L2_CID_TILT_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+5)
#define V4L2_CID_PAN_RESET      (V4L2_CID_CAMERA_CLASS_BASE+6)
#define V4L2_CID_TILT_RESET      (V4L2_CID_CAMERA_CLASS_BASE+7)

#define V4L2_CID_PAN_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+8)
#define V4L2_CID_TILT_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+9)

#define V4L2_CID_FOCUS_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+10)
#define V4L2_CID_FOCUS_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+11)
#define V4L2_CID_FOCUS_AUTO      (V4L2_CID_CAMERA_CLASS_BASE+12)
 
#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)*/

std::string controlName(unsigned int id)
{
	std::string name = "None";
	switch(id)
	{
	case V4L2_CID_BRIGHTNESS:
		name = "Brightness";
		break;
	case V4L2_CID_CONTRAST:
		name = "Contrast";
		break;
	case V4L2_CID_SATURATION:
		name = "Saturation";
		break;
	case V4L2_CID_HUE:
		name = "Hue";
		break;
	case V4L2_CID_RED_BALANCE:
		name = "Red Balance";
		break;
	case V4L2_CID_BLUE_BALANCE:
		name = "Blue Balance";
		break;
	case V4L2_CID_GAIN:
		name = "Gain";
		break;
	case V4L2_CID_EXPOSURE:
		name = "Exposure";
		break;
	case V4L2_CID_AUTOEXPOSURE:
		name = "Auto Exposure";
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		name = "Auto White Balance";
		break;
	case V4L2_CID_AUTOGAIN:
		name = "Auto Gain";
		break;
	case V4L2_CID_HFLIP:
		name = "Horizontal Flip";
		break;
	case V4L2_CID_VFLIP:
		name = "Vertical Flip";
		break;
	default:
		name = "Unknown";
	}
	return name;
}

NAOCamera::NAOCamera() :
currentBuf(0),
timeStamp(0),
storedTimeStamp(Platform->getTime())
{
#if DEBUG_NUCAMERA_VERBOSITY > 4
    debug << "NAOCamera::NAOCamera()" << endl;
#endif

    // Set current camera to unknown.
    m_settings.activeCamera = CameraSettings::UNKNOWN_CAMERA;

    //Read camera settings from file.
    CameraSettings fileSettings;
    fileSettings.LoadFromFile(CONFIG_DIR + string("Camera.cfg"));

    // Open device
    openCameraDevice("/dev/video0");

    // set to top camera
    setActiveCamera(CameraSettings::TOP_CAMERA);
    initialiseCamera();
    readCameraSettings();
    m_cameraSettings[0] = m_settings;
    //forceApplySettings(fileSettings);
    applySettings(fileSettings);

    // Set Bottom Camera
    setActiveCamera(CameraSettings::BOTTOM_CAMERA);
    initialiseCamera();
    readCameraSettings();
    m_cameraSettings[1] = m_settings;
    //forceApplySettings(fileSettings);
    applySettings(fileSettings);

    loadCameraOffset();

    // enable streaming
    setStreaming(true);
}

NAOCamera::~NAOCamera()
{
#if DEBUG_NUCAMERA_VERBOSITY > 4
    debug << "NAOCamera::~NAOCamera()" << endl;
#endif
  // disable streaming
  setStreaming(false);

  // unmap buffers
  for(int i = 0; i < frameBufferCount; ++i)
    munmap(mem[i], memLength[i]);
  
  // close the device
  close(fd);
  free(buf);
}

void NAOCamera::openCameraDevice(std::string device_name)
{
    // open device
    fd = open(device_name.c_str(), O_RDWR);
    #if DEBUG_NUCAMERA_VERBOSITY > 4
    if(fd != -1)
    {
        debug << "NAOCamera::NAOCamera(): " << device_name << " Opened Successfully." << endl;
    }
    else {
        debug << "NAOCamera::NAOCamera(): " << device_name << " Could Not Be Opened: " << strerror(errno) << endl;
    }
    #endif
}

void NAOCamera::setStreaming(bool streaming_on)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int instruction = streaming_on ? VIDIOC_STREAMON: VIDIOC_STREAMOFF;
    VERIFY(ioctl(fd, instruction, &type) != -1);
    debug << "NAOCamera: streaming - " << streaming_on << endl;
}

void NAOCamera::initialiseCamera()
{
    int returnValue;
    // set default parameters
    struct v4l2_control control;
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_CAM_INIT;
    control.value = 0;
    VERIFY(ioctl(fd, VIDIOC_S_CTRL, &control) >= 0);

    v4l2_std_id esid0 = (WIDTH == 320 ? 0x04000000UL : 0x08000000UL);
    returnValue = ioctl(fd, VIDIOC_S_STD, &esid0);

    #if DEBUG_NUCAMERA_VERBOSITY > 4
    if(returnValue)
    {
        debug << "NAOCamera::NAOCamera(): Error Setting Video Mode: " << strerror(errno) << endl;
    }
    else
    {
        debug << "NAOCamera::NAOCamera(): Video Mode set to " << (WIDTH == 320 ? "QVGA" : "VGA") << endl;
    }
    #else
    VERIFY(!returnValue);
    #endif

    // set format
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    returnValue = ioctl(fd, VIDIOC_S_FMT, &fmt);
    #if DEBUG_NUCAMERA_VERBOSITY > 4
    if(returnValue)
    {
        debug << "NAOCamera::NAOCamera(): Error Setting Format: " << strerror(errno) << endl;
    }
    else
    {
        debug << "NAOCamera::NAOCamera(): Format set" << endl;
    }
    #else
    VERIFY(!returnValue);
    #endif

    ASSERT(fmt.fmt.pix.sizeimage == SIZE);

    // set frame rate
    struct v4l2_streamparm fps;
    memset(&fps, 0, sizeof(struct v4l2_streamparm));
    fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    VERIFY(!ioctl(fd, VIDIOC_G_PARM, &fps));
    fps.parm.capture.timeperframe.numerator = 1;
    fps.parm.capture.timeperframe.denominator = 30;
    VERIFY(ioctl(fd, VIDIOC_S_PARM, &fps) != -1);

    // request buffers
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
    rb.count = frameBufferCount;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    VERIFY(ioctl(fd, VIDIOC_REQBUFS, &rb) != -1);

    // map the buffers
    buf = static_cast<struct v4l2_buffer*>(calloc(1, sizeof(struct v4l2_buffer)));
    for(int i = 0; i < frameBufferCount; ++i)
    {
        buf->index = i;
        buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf->memory = V4L2_MEMORY_MMAP;
        VERIFY(ioctl(fd, VIDIOC_QUERYBUF, buf) != -1);
        memLength[i] = buf->length;
        mem[i] = mmap(0, buf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf->m.offset);
        ASSERT(mem[i] != MAP_FAILED);
    }

    // queue the buffers
    for(int i = 0; i < frameBufferCount; ++i)
    {
        buf->index = i;
        buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf->memory = V4L2_MEMORY_MMAP;
        VERIFY(ioctl(fd, VIDIOC_QBUF, buf) != -1);
    }
}

bool NAOCamera::capturedNew()
{
  // requeue the buffer of the last captured image which is obselete now
  if(currentBuf) 
    VERIFY(ioctl(fd, VIDIOC_QBUF, currentBuf) != -1);
  
  // dequeue a frame buffer (this call blocks when there is no new image available) */
  VERIFY(ioctl(fd, VIDIOC_DQBUF, buf) != -1);

  ASSERT(buf->bytesused == SIZE);
  currentBuf = buf;
  timeStamp = storedTimeStamp + 33.0;
  storedTimeStamp = Platform->getTime();
  return true;
}

const unsigned char* NAOCamera::getImage() const
{
  ASSERT(currentBuf);
  return static_cast<unsigned char*>(mem[currentBuf->index]);
}

double NAOCamera::getTimeStamp() const
{
  ASSERT(currentBuf);
  return timeStamp;
}

CameraSettings::Camera NAOCamera::getCurrentCamera()
{
    return m_settings.activeCamera;
}

NUImage* NAOCamera::grabNewImage()
{
    while(!capturedNew());
    currentBufferedImage.MapYUV422BufferToImage(getImage(), WIDTH, HEIGHT);
    currentBufferedImage.m_timestamp = getTimeStamp();
    return &currentBufferedImage;
}

void NAOCamera::readCameraSettings()
{
    // read in current camera settings.
    m_settings.brightness = readSetting(V4L2_CID_BRIGHTNESS);
    m_settings.contrast = readSetting(V4L2_CID_CONTRAST);
    m_settings.saturation = readSetting(V4L2_CID_SATURATION);
    m_settings.hue = readSetting(V4L2_CID_HUE);
    m_settings.redChroma = readSetting(V4L2_CID_RED_BALANCE);
    m_settings.blueChroma = readSetting(V4L2_CID_BLUE_BALANCE);
    m_settings.gain = readSetting(V4L2_CID_GAIN);
    m_settings.exposure = readSetting(V4L2_CID_EXPOSURE);

    m_settings.autoExposure = readSetting(V4L2_CID_AUTOEXPOSURE);
    m_settings.autoWhiteBalance = readSetting(V4L2_CID_AUTO_WHITE_BALANCE);
    m_settings.autoGain = readSetting(V4L2_CID_AUTOGAIN);
}

CameraSettings::Camera NAOCamera::setActiveCamera(CameraSettings::Camera newCamera)
{
    // If no change required, do nothing.
    if(m_settings.activeCamera == newCamera) return m_settings.activeCamera;

    CameraSettings::Camera previous_camera = m_settings.activeCamera;
    int i2cFd = open("/dev/i2c-0", O_RDWR);
    ASSERT(i2cFd != -1);
    VERIFY(ioctl(i2cFd, 0x703, 8) == 0);
    VERIFY(i2c_smbus_read_byte_data(i2cFd, 170) >= 2); // at least Nao V3
    unsigned char cmd[2] = {newCamera, 0};
    VERIFY(i2c_smbus_write_block_data(i2cFd, 220, 1, cmd) != -1); // set camera
    close(i2cFd);

    // Display debug message.
    debug << "NAOCamera: Active camera changed: " << CameraSettings::cameraName(previous_camera);
    debug << " --> " << CameraSettings::cameraName(newCamera) << endl;

    int horzontal_flip, vertical_flip;
    int cameraSettingsIndex;
    switch(newCamera)
    {
        case CameraSettings::TOP_CAMERA:
            horzontal_flip = 1;
            vertical_flip = 1;
            cameraSettingsIndex = 0;
            break;
        case CameraSettings::BOTTOM_CAMERA:
            horzontal_flip = 0;
            vertical_flip = 0;
            cameraSettingsIndex = 1;
            break;
        default:
            horzontal_flip = 0;
            vertical_flip = 0;
            cameraSettingsIndex = -1;
            break;
    }
    if(cameraSettingsIndex != -1)
    {
        m_cameraSettings[!cameraSettingsIndex] = m_settings;
        m_settings = m_cameraSettings[cameraSettingsIndex];
    }

    m_settings.activeCamera = newCamera;

    VERIFY(applySetting(V4L2_CID_HFLIP, horzontal_flip));
    VERIFY(applySetting(V4L2_CID_VFLIP, vertical_flip));

    return m_settings.activeCamera;
}

int NAOCamera::readSetting(unsigned int id)
{
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
    return -1;
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return -1; // not available
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
    return -1; // not supported

  struct v4l2_control control_s;
  control_s.id = id;
  if(ioctl(fd, VIDIOC_G_CTRL, &control_s) < 0) 
    return -1;
  if(control_s.value == queryctrl.default_value)
    return -1;
  return control_s.value;
}

bool NAOCamera::applySetting(unsigned int id, int value)
{
  struct v4l2_queryctrl queryctrl;
  queryctrl.id = id;
  if(ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) < 0)
    return false;
  if(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return false; // not available
  if(queryctrl.type != V4L2_CTRL_TYPE_BOOLEAN && queryctrl.type != V4L2_CTRL_TYPE_INTEGER && queryctrl.type != V4L2_CTRL_TYPE_MENU)
    return false; // not supported

  // clip value
  if(value < queryctrl.minimum)
    value = queryctrl.minimum;
  if(value > queryctrl.maximum)
    value = queryctrl.maximum;
  if(value < 0)
    value = queryctrl.default_value;  

  struct v4l2_control control_s;
  control_s.id = id;
  control_s.value = value;
  if(ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0) 
    return false;
  debug << "NAOCamera: Apply (" << CameraSettings::cameraName(m_settings.activeCamera) << "): " << controlName(id) << " -> " << value << " (" << queryctrl.minimum << "," << queryctrl.maximum << "," << queryctrl.default_value << ")" << std::endl;
  return true;
}

void NAOCamera::applySettings(const CameraSettings& newset)
{
  // Auto Controls
  if(newset.autoExposure != m_settings.autoExposure)
    applySetting(V4L2_CID_AUTOEXPOSURE, (m_settings.autoExposure = newset.autoExposure));
  if(newset.autoWhiteBalance != m_settings.autoWhiteBalance)
    applySetting(V4L2_CID_AUTO_WHITE_BALANCE, (m_settings.autoWhiteBalance = newset.autoWhiteBalance));
  if(newset.autoGain != m_settings.autoGain)
    applySetting(V4L2_CID_AUTOGAIN, (m_settings.autoGain = newset.autoGain));

  if(newset.brightness != m_settings.brightness)
    applySetting(V4L2_CID_BRIGHTNESS, (m_settings.brightness = newset.brightness));
  if(newset.contrast != m_settings.contrast)
    applySetting(V4L2_CID_CONTRAST, (m_settings.contrast = newset.contrast));
  if(newset.saturation != m_settings.saturation)
    applySetting(V4L2_CID_SATURATION, (m_settings.saturation = newset.saturation));
  if(newset.hue != m_settings.hue)
    applySetting(V4L2_CID_HUE, (m_settings.hue = newset.hue));
  if(newset.redChroma != m_settings.redChroma)
    applySetting(V4L2_CID_RED_BALANCE, (m_settings.redChroma = newset.redChroma));
  if(newset.blueChroma != m_settings.blueChroma)
    applySetting(V4L2_CID_BLUE_BALANCE, (m_settings.blueChroma = newset.blueChroma));
  if(newset.gain != m_settings.gain)
    applySetting(V4L2_CID_GAIN, (m_settings.gain = newset.gain));
  if(newset.exposure != m_settings.exposure)
    applySetting(V4L2_CID_EXPOSURE, (m_settings.exposure = newset.exposure));
}

void NAOCamera::forceApplySettings(const CameraSettings& newset)
{
    // Auto Controls
    applySetting(V4L2_CID_AUTOEXPOSURE, (m_settings.autoExposure = newset.autoExposure));
    applySetting(V4L2_CID_AUTO_WHITE_BALANCE, (m_settings.autoWhiteBalance = newset.autoWhiteBalance));
    applySetting(V4L2_CID_AUTOGAIN, (m_settings.autoGain = newset.autoGain));

    applySetting(V4L2_CID_BRIGHTNESS, (m_settings.brightness = newset.brightness));
    applySetting(V4L2_CID_CONTRAST, (m_settings.contrast = newset.contrast));
    applySetting(V4L2_CID_SATURATION, (m_settings.saturation = newset.saturation));
    applySetting(V4L2_CID_HUE, (m_settings.hue = newset.hue));
    applySetting(V4L2_CID_RED_BALANCE, (m_settings.redChroma = newset.redChroma));
    applySetting(V4L2_CID_BLUE_BALANCE, (m_settings.blueChroma = newset.blueChroma));
    applySetting(V4L2_CID_GAIN, (m_settings.gain = newset.gain));
    applySetting(V4L2_CID_EXPOSURE, (m_settings.exposure = newset.exposure));
}

void NAOCamera::loadCameraOffset()
{
    ifstream file((CONFIG_DIR + string("CameraOffsets.cfg")).c_str());
    if (file.is_open())
    {
        string macaddress = Platform->getMacAddress();
        while (not file.eof())
        {
            string buffer;
            string addr_buffer;
            float offset_buffer;
            
            getline(file, buffer);
            stringstream ss(buffer);
            getline(ss, addr_buffer, ':');
            ss >> offset_buffer;
            
            if (macaddress.compare(addr_buffer) == 0)
            {
                CameraOffset = offset_buffer;
                break;
            }
        }
    }
    else
    {
        errorlog << "NAOCamera::loadCameraOffset(). Unable to load camera offset." << endl;
    }
}

