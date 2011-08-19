/*! @file NUOpenCVCamera.h
    @brief Declaration of a generic opencv--based camera class

    @class A generic opencv--based camera class.

    		This class uses opencv to access the camera. This has the nice property of working
    		anywhere opencv is available. However, the camera settings are limited.

    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#ifndef NUOPENCVCAMERA_H
#define NUOPENCVCAMERA_H

#include "Infrastructure/NUImage/NUImage.h"
#include "NUPlatform/NUCamera/CameraSettings.h"
#include "NUPlatform/NUCamera.h"

namespace cv {
	class VideoCapture;
}

class NUOpenCVCamera : public NUCamera
{
public:
    static float CameraOffset;
public:
    NUOpenCVCamera();
    ~NUOpenCVCamera();
    NUImage* grabNewImage();
    void setSettings(const CameraSettings& newset);
protected:
    cv::VideoCapture* m_camera;
    NUImage* m_image;
    unsigned char* m_yuyv_buffer;
};

#endif

