/*! @file DarwinWebotsCamera.cpp
    @brief Implementation of Darwin in Webots camera class

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

#include "DarwinWebotsCamera.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "Infrastructure/NUImage/Pixel.h"
#include "NUPlatform/NUPlatform.h"

#include "debug.h"
#include "debugverbositynucamera.h"

using namespace webots;

/*! @brief Constructs a webots camera
    @param platform a pointer to the platform (which in webots inherits from webots::Robot)
 */
DarwinWebotsCamera::DarwinWebotsCamera(DarwinWebotsPlatform* platform)
{
#if DEBUG_NUCAMERA_VERBOSITY > 0
    debug << "DarwinWebotsCamera::DarwinWebotsCamera(" << platform << ")" << endl;
#endif
    m_camera = platform->getCamera("camera");
    m_camera->enable(80); 
    
    m_width = m_camera->getWidth();
    m_height = m_camera->getHeight();
    m_totalpixels = m_width*m_height;

#if DEBUG_NUCAMERA_VERBOSITY > 0
    debug << "DarwinWebotsCamera::DarwinWebotsCamera(). Width = " << m_width << " Height = " << m_height << endl;
#endif
    
    m_image = new NUImage(m_width, m_height, false);
    m_yuyv_buffer = new Pixel[m_totalpixels*2];
}

/*! @brief Destory the DarwinWebotsCamera
 */
DarwinWebotsCamera::~DarwinWebotsCamera()
{
    if (m_camera != NULL)
    {
        m_camera->disable();
        delete m_camera;
    }
    if (m_image != NULL)
        delete m_image;
    if (m_yuyv_buffer != NULL)
        delete m_yuyv_buffer;
}

/*! @brief Returns a pointer to a new image.
 
    The image is 160 x 120, and is in the usual YUV422 format.
 */
NUImage* DarwinWebotsCamera::grabNewImage()
{
    const unsigned char* rgb_image = m_camera->getImage();              // grab the image from webots
    
    unsigned char y, u, v;
    int j,k;
    for (int i=0; i<m_totalpixels; i++)                                 // convert from rgb to yuv422
    {
        j = 3*i;
        ColorModelConversions::fromRGBToYCbCr(rgb_image[j], rgb_image[j + 1], rgb_image[j + 2], y, u, v);
        m_yuyv_buffer[i].yCbCrPadding = y;
        m_yuyv_buffer[i].cb = u;
        m_yuyv_buffer[i].y = y;
        m_yuyv_buffer[i].cr = v;
    }
    
    m_image->MapBufferToImage(m_yuyv_buffer, m_width, m_height);  // have NUImage use m_yuyv_buffer
    m_image->setTimestamp(Platform->getTime());
    return m_image;
}

/*! @brief The DarwinWebotsCamera has no settings, so this function does nothing
 */
void DarwinWebotsCamera::setSettings(const CameraSettings& newset)
{
}




