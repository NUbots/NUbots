/*! @file BearCamera.cpp
    @brief Implementation of Bear camera class

    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
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

#include "BearCamera.h"

#include "debug.h"
#include "debugverbositynucamera.h"

/*! @brief Constructs a webots camera
 */
BearCamera::BearCamera()
{
#if DEBUG_NUCAMERA_VERBOSITY > 0
    debug << "BearCamera::BearCamera(" << platform << ")" << std::endl;
#endif
}

/*! @brief Destory the BearCamera
 */
BearCamera::~BearCamera()
{
}

/*! @brief Returns a pointer to a new image.
 */
NUImage* BearCamera::grabNewImage()
{
    // this might be bad because the SeeThinkThread will have an infinite loop frequency
    return NULL;
}

/*! @brief The BearCamera has no settings, so this function does nothing
 */
void BearCamera::setSettings(const CameraSettings& newset)
{
}




