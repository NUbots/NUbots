/*! @file ChangeCameraSettingsJob.h
    @brief Declaration of ChangeCameraSettingsJob class.
 
    @class ChangeCameraSettingsJob
    @brief A job to set whether images should be saved or not.
 
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

#ifndef CHANGECAMERASETTINGSJOB_H
#define CHANGECAMERASETTINGSJOB_H

#include "../CameraJob.h"
#include "NUPlatform/NUCamera/CameraSettings.h"

class ChangeCameraSettingsJob : public CameraJob
{
public:
    ChangeCameraSettingsJob(const CameraSettings& settings);
    ChangeCameraSettingsJob(std::istream& input);
    virtual ~ChangeCameraSettingsJob();
    
    CameraSettings& getSettings();
    void setSettings(const CameraSettings& newsettings);
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, const ChangeCameraSettingsJob& job);
    friend std::ostream& operator<<(std::ostream& output, const ChangeCameraSettingsJob* job);
protected:
    virtual void toStream(std::ostream& output) const;
private:
    CameraSettings m_camera_settings;         //!< the camera settings to apply
};

#endif

