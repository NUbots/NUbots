/*! @file DarwinCamera.cpp
    @brief Implementation of Darwin camera class

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

#include "DarwinCamera.h"
#include "NUPlatform/NUPlatform.h"

#include "debug.h"
#include "debugverbositynucamera.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"
using namespace cv;

/*! @brief Constructs a webots camera
 */
/*DarwinCamera::DarwinCamera()
{
    #if DEBUG_NUCAMERA_VERBOSITY > 0
        debug << "DarwinCamera::DarwinCamera(" << platform << ")" << endl;
    #endif
}*/

/*! @brief Destory the DarwinCamera
 */
/*DarwinCamera::~DarwinCamera()
{
}*/

NUImage* DarwinCamera::grabNewImage()
{
	Mat frame;
	if (m_camera)
	{
		*m_camera >> frame;
		size_t size = frame.rows*frame.cols;
		if (not m_yuyv_buffer or frame.rows != m_image->getHeight() or frame.cols != m_image->getWidth())
		{
			delete m_yuyv_buffer;
			m_yuyv_buffer = new unsigned char[size*2];
			m_image->MapYUV422BufferToImage(m_yuyv_buffer, frame.cols, frame.rows);
		}

		int i_YUV = 0;			// the index into the yuyv_buffer
		unsigned char y1,u1,v1,y2,u2,v2;
		for (size_t i=frame.rows; i>0; i--)
		{
			for (size_t j=frame.cols/2; j>0; j--)		// count down to flip the image around
			{
				ColorModelConversions::fromRGBToYCbCr(frame.at<Vec3b>(i,j<<1)[2], frame.at<Vec3b>(i,j<<1)[1], frame.at<Vec3b>(i,j<<1)[0], y1, u1, v1);
				ColorModelConversions::fromRGBToYCbCr(frame.at<Vec3b>(i,(j<<1)-1)[2], frame.at<Vec3b>(i,(j<<1)-1)[1], frame.at<Vec3b>(i,(j<<1)-1)[0], y2, u2, v2);
				m_yuyv_buffer[i_YUV++] = y1;
				m_yuyv_buffer[i_YUV++] = (u1+u2)>>1;
				m_yuyv_buffer[i_YUV++] = y2;
				m_yuyv_buffer[i_YUV++] = (v1+v2)>>1;
			}
		}
	}
        m_image->setTimestamp(Platform->getTime());
	return m_image;
}


