/*! @file NUCamera.h
    @brief Declaration of a base camera class

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

#ifndef NUCAMERA_H
#define NUCAMERA_H

#include "NUPlatform/NUActionators.h"
#include "Tools/Image/NUimage.h"
#include "NUCamera/CameraSettings.h"

class NUCamera
{
public:
    static float CameraOffset;
public:
    virtual ~NUCamera();
    virtual NUimage* grabNewImage() = 0;
    virtual void setSettings(const CameraSettings& newset) = 0;
    CameraSettings getSettings(){return settings;};
    
protected:
    CameraSettings settings;
private:
};

#endif

