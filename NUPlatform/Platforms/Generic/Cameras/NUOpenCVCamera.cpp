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

#include "opencv/cv.h"
#include "opencv/highgui.h"
using namespace cv;

#include "debug.h"

float NUOpenCVCamera::CameraOffset = 0;

NUOpenCVCamera::NUOpenCVCamera()
{
	m_camera = new VideoCapture(0); 			// open the default camera
	if(m_camera or !m_camera->isOpened())  	// check if we succeeded
	{
		debug << "NUOpenCVCamera::NUOpenCVCamera(). Failed to open default camera." << endl;
		errorlog << "NUOpenCVCamera::NUOpenCVCamera(). Failed to open default camera." << endl;
		m_camera = 0;
	}
}

NUOpenCVCamera::~NUOpenCVCamera()
{
	delete m_camera;
	m_camera = 0;
}

NUImage* NUOpenCVCamera::grabNewImage()
{
	return NULL;
}

void NUOpenCVCamera::setSettings(const CameraSettings& newset)
{

}



