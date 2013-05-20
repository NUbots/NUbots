/**
*       @name PCCamera
*       @file pccamera.h
*       @brief V4l wrapper for the PC webcam to enable retrieval in native YUYV format
*       @author Shannon Fenn
*       @author David Budden
*       @date 21-02-12
*
*/

#ifndef PCCAMERA_H
#define PCCAMERA_H

#include "NUPlatform/NUCamera/CameraSettings.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "NUPlatform/NUCamera.h"

#define DEBUG_NUCAMERA_VERBOSITY 0

#define CAMERA_DIR "/dev/video1"

class PCCamera : public NUCamera
{
public:
    enum
      {
          frameBufferCount = 20, //!< Number of available frame buffers.
          WIDTH = 640,
          HEIGHT = 480,
          SIZE = WIDTH * HEIGHT * 2
      };

    PCCamera();
    ~PCCamera();
    NUImage* grabNewImage();
    unsigned char* grabNewImageBuffer();
    void applySettings(const CameraSettings& newset);
    void setSettings(const CameraSettings& newset){
        applySettings(newset);
    }
    void forceApplySettings(const CameraSettings& newset);
    CameraSettings getSettings(){return m_settings;};

private:

    bool applySetting(unsigned int settingID, int value);
    int readSetting(unsigned int id);
    void initialiseCamera();
    void readCameraSettings();
    void openCameraDevice(std::string device_name);
    void setStreaming(bool streaming_on);

    int fd;                             //!< The file descriptor for the video device.
    void* mem[frameBufferCount];        //!< Frame buffer addresses.
    int memLength[frameBufferCount]; 	//!< The length of each frame buffer.
    struct v4l2_buffer* buf;            //!< Reusable parameter struct for some ioctl calls.
    struct v4l2_buffer* currentBuf; 	//!< The last dequeued frame buffer.
    bool capturedNew();
    unsigned char* getImage() const;

    NUImage currentBufferedImage;
protected:
    CameraSettings m_settings;
};

#endif // PCCAMERA_H
