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

#include <cstring>
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
 
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26) */

#include "NAOCamera.h"
#include "NUPlatform/NUSystem.h"
#include "GTAssert.h"
#include "debug.h"
#include "debugverbositynucamera.h"

NAOCamera::NAOCamera() :
currentBuf(0),
timeStamp(0),
storedTimeStamp(nusystem->getTime())
{
#if DEBUG_NUCAMERA_VERBOSITY > 4
    debug << "NAOCamera::NAOCamera()" << endl;
#endif
  int returnValue;

  int i2cFd = open("/dev/i2c-0", O_RDWR);
  ASSERT(i2cFd != -1);
  VERIFY(ioctl(i2cFd, 0x703, 8) == 0);
  VERIFY(i2c_smbus_read_byte_data(i2cFd, 170) >= 2); // at least Nao V3
  unsigned char cmd[2] = {2, 0};
  VERIFY(i2c_smbus_write_block_data(i2cFd, 220, 1, cmd) != -1); // select lower camera
  close(i2cFd);
    settings.activeCamera = 1;

  // open device
  fd = open("/dev/video0", O_RDWR);
    #if DEBUG_NUCAMERA_VERBOSITY > 4
    if(fd != -1)
    {
        debug << "NAOCamera::NAOCamera(): /dev/video0 Opened Successfully." << endl;  
    }
    else {
        debug << "NAOCamera::NAOCamera(): /dev/video0 Could Not Be Opened: " << strerror(errno) << endl;  
    }   
    #else 
    VERIFY(fd != -1);
    #endif



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

  // request camera's default control settings
  settings.brightness = getControlSetting(V4L2_CID_BRIGHTNESS);
  //settings.contrast = getControlSetting(?);
  //settings.saturation = getControlSetting(?);
  //settings.hue = getControlSetting(?);
  settings.redChroma = getControlSetting(V4L2_CID_RED_BALANCE); 
  settings.blueChroma = getControlSetting(V4L2_CID_BLUE_BALANCE); 
  settings.gain = getControlSetting(V4L2_CID_GAIN);
  settings.exposure = getControlSetting(V4L2_CID_EXPOSURE);
  // make sure automatic stuff is off

    VERIFY(setControlSetting(V4L2_CID_AUTOEXPOSURE , 0));
    settings.autoExposure = 0;
    VERIFY(setControlSetting(V4L2_CID_AUTO_WHITE_BALANCE, 0));
    settings.autoWhiteBalance = 0;
    VERIFY(setControlSetting(V4L2_CID_AUTOGAIN, 0));
    settings.autoGain = 0;
    VERIFY(setControlSetting(V4L2_CID_HFLIP, 0));
    VERIFY(setControlSetting(V4L2_CID_VFLIP, 0));

  // enable streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMON, &type) != -1);
  
  	//READ CAMERA SETTINGS FROM FILE:

	CameraSettings fileSettings;
	fileSettings.LoadFromFile("/home/nao/data/Camera.cfg");
	setSettings(fileSettings);
	
}

NAOCamera::~NAOCamera()
{
#if DEBUG_NUCAMERA_VERBOSITY > 4
    debug << "NAOCamera::~NAOCamera()" << endl;
#endif
  // disable streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  VERIFY(ioctl(fd, VIDIOC_STREAMOFF, &type) != -1);

  // unmap buffers
  for(int i = 0; i < frameBufferCount; ++i)
    munmap(mem[i], memLength[i]);
  
  // close the device
  close(fd);
  free(buf);
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
  storedTimeStamp = nusystem->getTime();
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

NUimage* NAOCamera::grabNewImage()
{
    while(!capturedNew());
    currentBufferedImage.MapYUV422BufferToImage(getImage(), WIDTH, HEIGHT);
    currentBufferedImage.m_timestamp = getTimeStamp();
    return &currentBufferedImage;
}

int NAOCamera::getControlSetting(unsigned int id)
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

bool NAOCamera::setControlSetting(unsigned int id, int value)
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
  return true;
}

void NAOCamera::setSettings(const CameraSettings& newset)
{
  if(newset.brightness != settings.brightness)
    setControlSetting(V4L2_CID_BRIGHTNESS, (settings.brightness = newset.brightness));
//  if(newset.contrast != settings.contrast)
//    setControlSetting(?, (settings.contrast = newset.contrast));
//  if(newset.saturation != settings.saturation)
//    setControlSetting(?, (settings.saturation = newset.saturation));
//  if(newset.hue != settings.hue)
//    setControlSetting(?, (settings.hue = newset.hue));
  if(newset.redChroma != settings.redChroma)
    setControlSetting(V4L2_CID_RED_BALANCE, (settings.redChroma = newset.redChroma));
  if(newset.blueChroma != settings.blueChroma)
    setControlSetting(V4L2_CID_BLUE_BALANCE, (settings.blueChroma = newset.blueChroma));
  if(newset.gain != settings.gain)
    setControlSetting(V4L2_CID_GAIN, (settings.gain = newset.gain));
  if(newset.exposure != settings.exposure)
    setControlSetting(V4L2_CID_EXPOSURE, (settings.exposure = newset.exposure));

// Auto Controls
  if(newset.autoExposure != settings.autoExposure)
    setControlSetting(V4L2_CID_AUTOEXPOSURE, (settings.autoExposure = newset.autoExposure));
  if(newset.autoWhiteBalance != settings.autoWhiteBalance)
    setControlSetting(V4L2_CID_AUTO_WHITE_BALANCE, (settings.autoWhiteBalance = newset.autoWhiteBalance));
  if(newset.autoGain != settings.autoGain)
    setControlSetting(V4L2_CID_AUTOGAIN, (settings.autoGain = newset.autoGain));
}

