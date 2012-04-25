#include "pccamera.h"

#include "debug.h"
#include "GTAssert.h"

#include <cstring>
#include <sstream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdlib.h>

//ORIGINAL
/*
#undef __STRICT_ANSI__
#include <linux/videodev.h>
#include <linux/version.h>
#include <bn/i2c/i2c-dev.h>
#define __STRICT_ANSI__
*/
#undef __STRICT_ANSI__
#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/i2c-dev.h>
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

std::string controlName(unsigned int id)
{
        std::string name;
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
        case V4L2_CID_AUTO_WHITE_BALANCE:
            name = "Auto White Balance";
            break;
        case V4L2_CID_POWER_LINE_FREQUENCY:
            name = "Power Line Frequency";
            break;
        case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
            name = "White Balance Temperature";
            break;
        case V4L2_CID_SHARPNESS:
            name = "Sharpness";
            break;
        case V4L2_CID_EXPOSURE_AUTO:
            name = "Exposure Auto";
            break;
        case V4L2_CID_EXPOSURE_ABSOLUTE:
            name = "Absolute Exposure";
            break;
        case V4L2_CID_EXPOSURE_AUTO_PRIORITY:
            name = "Exposure Auto Priority";
            break;
        case V4L2_CID_GAIN:
            name = "Gain";
            break;
        default:
            name = "Unknown";
        }
        return name;
}

PCCamera::PCCamera() :
currentBuf(0)
{
    #if DEBUG_NUCAMERA_VERBOSITY > 4
        debug << "PCCamera::PCCamera()" << endl;
    #endif

    //Read camera settings from file.
    CameraSettings fileSettings;
    fileSettings.LoadFromFile(CONFIG_DIR + string("Camera.cfg"));
    #if DEBUG_NUCAMERA_VERBOSITY > 4
        debug << "Loading settings from " << CONFIG_DIR + string("Camera.cfg") << endl;
    #endif

    // Open device
    openCameraDevice(CAMERA_DIR);

    //Initialise
    initialiseCamera();
    //readCameraSettings();
    forceApplySettings(fileSettings);

    readCameraSettings();

    // enable streaming
    setStreaming(true);
}

PCCamera::~PCCamera()
{
#if DEBUG_NUCAMERA_VERBOSITY > 4
    debug << "PCCamera::~PCCamera()" << endl;
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

void PCCamera::openCameraDevice(std::string device_name)
{
    // open device
    fd = open(device_name.c_str(), O_RDWR);
    #if DEBUG_NUCAMERA_VERBOSITY > 4
    if(fd != -1)
    {
        debug << "PCCamera::PCCamera(): " << device_name << " Opened Successfully." << endl;
    }
    else {
        debug << "PCCamera::PCCamera(): " << device_name << " Could Not Be Opened: " << strerror(errno) << endl;
    }
    #endif
    if(fd == -1)
        errorlog << "PCCamera::PCCamera(): " << device_name << " Could Not Be Opened: " << strerror(errno) << endl;
    ASSERT(fd >= 0);
}

void PCCamera::setStreaming(bool streaming_on)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int instruction = streaming_on ? VIDIOC_STREAMON: VIDIOC_STREAMOFF;
    VERIFY(ioctl(fd, instruction, &type) != -1);
#if DEBUG_NUCAMERA_VERBOSITY > 1
    debug << "PCCamera: streaming - " << streaming_on << endl;
#endif
}

void PCCamera::initialiseCamera()
{
    int returnValue;

    //set default parameters
//    struct v4l2_control control;
//    memset(&control, 0, sizeof(control));
//    control.id = V4L2_CID_CAM_INIT;
//    control.value = 0;

//    //returnValue = ioctl(fd, VIDIOC_S_CTRL, &control);
//    returnValue = ioctl(fd, VIDIOC_S_EXT_CTRLS, &control);
//    VERIFY(returnValue >= 0);

    //CANNOT SET THE VIDEO MODE FOR USB CAMERAS

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
        debug << "PCCamera::PCCamera(): Error Setting Format: " << strerror(errno) << endl;
    }
    else
    {
        debug << "PCCamera::PCCamera(): Format set" << endl;
    }
    #else
    //VERIFY(!returnValue);
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

bool PCCamera::capturedNew()
{
  // requeue the buffer of the last captured image which is obselete now
  if(currentBuf)
    VERIFY(ioctl(fd, VIDIOC_QBUF, currentBuf) != -1);

  // dequeue a frame buffer (this call blocks when there is no new image available) */
  VERIFY(ioctl(fd, VIDIOC_DQBUF, buf) != -1);

  ASSERT(buf->bytesused == SIZE);
  currentBuf = buf;
  return true;
}

unsigned char* PCCamera::getImage() const
{
  ASSERT(currentBuf);
  return static_cast<unsigned char*>(mem[currentBuf->index]);
}

NUImage* PCCamera::grabNewImage()
{
    while(!capturedNew());
    currentBufferedImage.MapYUV422BufferToImage(getImage(), WIDTH, HEIGHT, false);
    currentBufferedImage.setCameraSettings(m_settings);
    return &currentBufferedImage;
}

unsigned char* PCCamera::grabNewImageBuffer()
{
    while(!capturedNew());
    return getImage();
}

void PCCamera::readCameraSettings()
{
    m_settings.exposureAuto               = readSetting(V4L2_CID_EXPOSURE_AUTO);
    m_settings.autoWhiteBalance           = readSetting(V4L2_CID_AUTO_WHITE_BALANCE);
    m_settings.exposureAutoPriority       = readSetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY);
    m_settings.brightness                 = readSetting(V4L2_CID_BRIGHTNESS);
    m_settings.contrast                   = readSetting(V4L2_CID_CONTRAST);
    m_settings.saturation                 = readSetting(V4L2_CID_SATURATION);
    m_settings.gain                       = readSetting(V4L2_CID_GAIN);
    m_settings.exposureAbsolute           = readSetting(V4L2_CID_EXPOSURE_ABSOLUTE);
    m_settings.powerLineFrequency         = readSetting(V4L2_CID_POWER_LINE_FREQUENCY);
    m_settings.sharpness                  = readSetting(V4L2_CID_SHARPNESS);

    m_settings.p_exposureAuto.set(m_settings.exposureAuto);
    m_settings.p_autoWhiteBalance.set(m_settings.autoWhiteBalance);
    m_settings.p_exposureAutoPriority.set(m_settings.exposureAutoPriority);
    m_settings.p_brightness.set(m_settings.brightness);
    m_settings.p_contrast.set(m_settings.contrast);
    m_settings.p_saturation.set(m_settings.saturation);
    m_settings.p_gain.set(m_settings.gain);
    m_settings.p_exposureAbsolute.set(m_settings.exposureAbsolute);
    m_settings.p_powerLineFrequency.set(m_settings.powerLineFrequency);
    m_settings.p_sharpness.set(m_settings.sharpness);

#if DEBUG_NUCAMERA_VERBOSITY > 1
    debug << "PCCamera::readCameraSettings()" << endl;
    debug << "\texposureAuto " << m_settings.exposureAuto  << endl;
    debug << "autoWhiteBalance " << m_settings.autoWhiteBalance  << endl;
    debug << "exposureAutoPriority " << m_settings.exposureAutoPriority  << endl;
    debug << "brightness " << m_settings.brightness  << endl;
    debug << "contrast " << m_settings.contrast  << endl;
    debug << "saturation " << m_settings.saturation  << endl;
    debug << "gain " << m_settings.gain  << endl;
    debug << "exposureAbsolute " << m_settings.exposureAbsolute  << endl;
    debug << "powerLineFrequency " << m_settings.powerLineFrequency  << endl;
    debug << "sharpness " << m_settings.sharpness  << endl;
#endif
}

int PCCamera::readSetting(unsigned int id)
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

bool PCCamera::applySetting(unsigned int id, int value)
{
#if DEBUG_NUCAMERA_VERBOSITY > 1
    debug << "PCCamera: Trying: " << controlName(id) << " -> " << value << std::endl;
#endif
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

    struct v4l2_control control_s;
    control_s.id = id;
    control_s.value = value;
    if(ioctl(fd, VIDIOC_S_CTRL, &control_s) < 0)
        return false;
#if DEBUG_NUCAMERA_VERBOSITY > 1
    debug << "PCCamera: Apply: " << controlName(id) << " -> " << value << " (" << queryctrl.minimum << "," << queryctrl.maximum << "," << queryctrl.default_value << ")" << std::endl;
#endif
    return true;
}

void PCCamera::applySettings(const CameraSettings& newset)
{
    // AutoSettings
    if(newset.p_exposureAuto.get() != m_settings.p_exposureAuto.get())
    {
        m_settings.p_exposureAuto.set(newset.p_exposureAuto.get());
        applySetting(V4L2_CID_EXPOSURE_AUTO, (m_settings.p_exposureAuto.get()));
    }
    if(newset.p_autoWhiteBalance.get() != m_settings.p_autoWhiteBalance.get())
    {
        m_settings.p_autoWhiteBalance.set(newset.p_autoWhiteBalance.get());
        applySetting(V4L2_CID_AUTO_WHITE_BALANCE, (m_settings.p_autoWhiteBalance.get()));
    }
    if(newset.p_exposureAutoPriority.get() != m_settings.p_exposureAutoPriority.get())
    {
        m_settings.p_exposureAutoPriority.set(newset.p_exposureAutoPriority.get());
        applySetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY, (m_settings.p_exposureAutoPriority.get()));
    }

    //Other controls
    if(newset.p_brightness.get() != m_settings.p_brightness.get())
    {
        m_settings.p_brightness.set(newset.p_brightness.get());
        applySetting(V4L2_CID_BRIGHTNESS, (m_settings.p_brightness.get()));
    }
    if(newset.p_contrast.get() != m_settings.p_contrast.get())
    {
        m_settings.p_contrast.set(newset.p_contrast.get());
        applySetting(V4L2_CID_CONTRAST, (m_settings.p_contrast.get()));
    }
    if(newset.p_saturation.get() != m_settings.p_saturation.get())
    {
        m_settings.p_saturation.set(newset.p_saturation.get());
        applySetting(V4L2_CID_SATURATION, (m_settings.p_saturation.get()));
    }
    if(newset.p_gain.get() != m_settings.p_gain.get())
    {
        m_settings.p_gain.set(newset.p_gain.get());
        applySetting(V4L2_CID_GAIN, (m_settings.p_gain.get()));
    }
    if(newset.p_exposureAbsolute.get() != m_settings.p_exposureAbsolute.get())
    {
        m_settings.p_exposureAbsolute.set(newset.p_exposureAbsolute.get());
        applySetting(V4L2_CID_EXPOSURE_ABSOLUTE, (m_settings.p_exposureAbsolute.get()));
    }

    if(newset.p_powerLineFrequency.get() != m_settings.p_powerLineFrequency.get())
    {
        m_settings.p_powerLineFrequency.set(newset.p_powerLineFrequency.get());
        applySetting(V4L2_CID_POWER_LINE_FREQUENCY, (m_settings.p_powerLineFrequency.get()));
    }
    if(newset.p_sharpness.get() != m_settings.p_sharpness.get())
    {
        m_settings.p_sharpness.set(newset.p_sharpness.get());
        applySetting(V4L2_CID_SHARPNESS, (m_settings.p_sharpness.get()));
    }

    //COPIES INTO OLD FORMAT:
    m_settings.copyParams();
}

void PCCamera::forceApplySettings(const CameraSettings& newset)
{
#if DEBUG_NUCAMERA_VERBOSITY > 1
    //Copying the new Paramters into m_settings
    debug << "p_exposureAuto" << newset.p_exposureAuto.get() << endl;
    debug << "p_autoWhiteBalance" << newset.p_autoWhiteBalance.get() << endl;
    debug << "p_exposureAutoPriority" << newset.p_exposureAutoPriority.get() << endl;
    debug << "p_brightness" << newset.p_brightness.get() << endl;
    debug << "p_contrast" << newset.p_contrast.get() << endl;
    debug << "p_saturation" << newset.p_saturation.get() << endl;
    debug << "p_gain" << newset.p_gain.get() << endl;
    debug << "p_exposureAbsolute" << newset.p_exposureAbsolute.get() << endl;
    debug << "p_powerLineFrequency" << newset.p_powerLineFrequency.get() << endl;
    debug << "p_sharpness" << newset.p_sharpness.get() << endl;
#endif
    //Copying the new Paramters into m_settings
    m_settings = newset;

    // Auto Controls
    applySetting(V4L2_CID_EXPOSURE_AUTO, (m_settings.p_exposureAuto.get()));
    applySetting(V4L2_CID_AUTO_WHITE_BALANCE, (m_settings.p_autoWhiteBalance.get()));
    applySetting(V4L2_CID_EXPOSURE_AUTO_PRIORITY, (m_settings.p_exposureAutoPriority.get()));
    //Other controls
    applySetting(V4L2_CID_BRIGHTNESS, (m_settings.p_brightness.get()));
    applySetting(V4L2_CID_CONTRAST, (m_settings.p_contrast.get()));
    applySetting(V4L2_CID_SATURATION, (m_settings.p_saturation.get()));
    applySetting(V4L2_CID_GAIN, (m_settings.p_gain.get()));
    applySetting(V4L2_CID_EXPOSURE_ABSOLUTE, (m_settings.p_exposureAbsolute.get()));
    applySetting(V4L2_CID_POWER_LINE_FREQUENCY, (m_settings.p_powerLineFrequency.get()));
    applySetting(V4L2_CID_SHARPNESS, (m_settings.p_sharpness.get()));

    //COPIES INTO OLD FORMAT:
    m_settings.copyParams();
}
