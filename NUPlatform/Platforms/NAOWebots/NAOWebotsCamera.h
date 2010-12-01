/*! @file NAOWebotsCamera.h
    @brief Declaration of NAO in Webots camera class

    @author Jason Kulk
 
    @class NAOWebotsCamera
    @brief A NAO in Webots camera
 
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

#ifndef NAOWEBOTSCAMERA_H
#define NAOWEBOTSCAMERA_H

#include "NUPlatform/NUCamera.h"
#include "NAOWebotsPlatform.h"
#include <cstring>

class webots::Camera;
class NUImage;

class NAOWebotsCamera : public NUCamera
{
public:
    NAOWebotsCamera(NAOWebotsPlatform* platform);
    ~NAOWebotsCamera();
    
    NUImage* grabNewImage();
    void setSettings(const CameraSettings& newset);
private:
    webots::Camera* m_camera;
    int m_width, m_height, m_totalpixels;
    
    NUImage* m_image;
    Pixel* m_yuyv_buffer;
};

#endif

