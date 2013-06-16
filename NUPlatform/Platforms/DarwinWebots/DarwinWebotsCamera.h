/*! @file DarwinWebotsCamera.h
    @brief Declaration of Darwin in Webots camera class

    @author Jason Kulk
 
    @class DarwinWebotsCamera
    @brief A Darwin in Webots camera
 
    @author Jason Kulk, Jed Rietveld
 
  Copyright (c) 2012 Jason Kulk, Jed Rietveld
 
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

#ifndef DARWINWEBOTSCAMERA_H
#define DARWINWEBOTSCAMERA_H

#include "NUPlatform/NUCamera.h"
#include "DarwinWebotsPlatform.h"
#include <cstring>

class NUImage;

class DarwinWebotsCamera : public NUCamera
{
public:
    DarwinWebotsCamera(DarwinWebotsPlatform* platform);
    ~DarwinWebotsCamera();
    
    NUImage* grabNewImage();
    void setSettings(const CameraSettings& newset);
private:
    webots::Camera* m_camera;
    int m_width, m_height, m_totalpixels;
    
    NUImage* m_image;
    Pixel* m_yuyv_buffer;
    friend class NUImage;
};

#endif

