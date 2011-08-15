/*! @file NUOpenCVCamera.cpp
    @brief Implementation of generic opencv camera class

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

#include "NUOpenCVCamera.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"
using namespace cv;

#include "debug.h"

float NUOpenCVCamera::CameraOffset = 0;

NUOpenCVCamera::NUOpenCVCamera()
{
	m_camera = new VideoCapture(0); 			// open the default camera
	if(not m_camera or !m_camera->isOpened())  	// check if we succeeded
	{
		debug << "NUOpenCVCamera::NUOpenCVCamera(). Failed to open default camera." << endl;
		errorlog << "NUOpenCVCamera::NUOpenCVCamera(). Failed to open default camera." << endl;
		m_camera = 0;
	}
	m_image = new NUImage();
	m_yuyv_buffer = 0;
}

NUOpenCVCamera::~NUOpenCVCamera()
{
	delete m_camera;
	m_camera = 0;
	delete m_image;
	m_image = 0;
	delete m_yuyv_buffer;
	m_yuyv_buffer = 0;
}

NUImage* NUOpenCVCamera::grabNewImage()
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
		for (size_t i=0; i<frame.rows; i++)
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
	return m_image;
}

void NUOpenCVCamera::setSettings(const CameraSettings& newset)
{

}



