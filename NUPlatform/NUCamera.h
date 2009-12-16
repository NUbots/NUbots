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

#include "NUPlatform/NUActionators/NUActionatorsData.h"
#include "Tools/Image/NUimage.h"

class NUCamera: public NUActionators
{
public:
	enum setting
	{
		Brightness,
		Contrast,
		Saturation,
		Hue,
		RedChroma,
		BlueChroma,
		Gain,
		AutoExposition,
		AutoWhiteBalance,
		AutoGain,
		Exposure,
		FramesPerSecond,
		CameraSelect
	};

    virtual ~NUCamera();
    virtual NUimage grabNewImage();
    virtual void setControlSetting(unsigned int settingID, int value);
    virtual int getControlSetting(unsigned int id);
};

#endif

