/*! @file NAOWebotsCamera.cpp
    @brief Implementation of NAO in Webots camera class

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

#include "NAOWebotsCamera.h"
#include "Tools/Image/ColorModelConversions.h"
#include "Tools/Image/Pixel.h"
#include "debug.h"
#include "debugverbositynucamera.h"

#include "webots/Robot.hpp"
using namespace webots;

/*! @brief Constructs a webots camera
    @param platform a pointer to the platform (which in webots inherits from webots::Robot)
 */
NAOWebotsCamera::NAOWebotsCamera(NAOWebotsPlatform* platform)
{
#if DEBUG_NUCAMERA_VERBOSITY > 0
    debug << "NAOWebotsCamera::NAOWebotsCamera(" << platform << ")" << endl;
#endif
    m_camera = platform->getCamera("camera");
    m_camera->enable(80); 
    
    platform->getServo("CameraSelect")->setPosition(0.6981);
    
    m_width = m_camera->getWidth();
    m_height = m_camera->getHeight();
    m_totalpixels = m_width*m_height;

#if DEBUG_NUCAMERA_VERBOSITY > 0
    debug << "NAOWebotsCamera::NAOWebotsCamera(). Width = " << m_width << " Height = " << m_height << endl;
#endif
    
    m_image = new NUimage(m_width, m_height, false);
    m_yuyv_buffer = new unsigned char[m_totalpixels*2];
}

/*! @brief Destory the NAOWebotsCamera
 */
NAOWebotsCamera::~NAOWebotsCamera()
{
    if (m_camera != NULL)
    {
        m_camera->disable();
        delete m_camera;
    }
}

/*! @brief Returns a pointer to a new image.
 
    The image is 160 x 120, and is in the usual YUV422 format.
 */
NUimage* NAOWebotsCamera::grabNewImage()
{
    const unsigned char* rgb_image = m_camera->getImage();              // grab the image from webots
    
    unsigned char y1, u1, v1, y2, u2, v2;
    int j,k;
    for (int i=0; i<m_totalpixels; i=i+2)                               // convert from rgb to yuv422
    {
        j = 3*i;
        ColorModelConversions::fromRGBToYCbCr(rgb_image[j], rgb_image[j + 1], rgb_image[j + 2], y1, u1, v1);
        ColorModelConversions::fromRGBToYCbCr(rgb_image[j + 3], rgb_image[j + 4], rgb_image[j + 5], y2, u2, v2);
        k = 2*i;
        m_yuyv_buffer[k] = y1;
        m_yuyv_buffer[k + 1] = (u1 + u2)/2;
        m_yuyv_buffer[k + 2] = y2;
        m_yuyv_buffer[k + 3] = (v1 + v2)/2;
    }
    m_image->MapYUV422BufferToImage(m_yuyv_buffer, m_width, m_height);  // have nuimage use m_yuyv_buffer
    
    debug << m_image->getWidth() << " " << m_image->getHeight() << endl;
    return m_image;
}

/*! @brief The NAOWebotsCamera has no settings, so this function does nothing
 */
void NAOWebotsCamera::setSettings(const CameraSettings& newset)
{
}




