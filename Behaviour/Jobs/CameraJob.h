/*! @file CameraJob.h
    @brief Declaration of camera job class.
 
    @class CameraJob
    @brief A class to encapsulate jobs issued by behaviour for the camera.
    
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

#ifndef CAMERAJOB_H
#define CAMERAJOB_H

#include "Behaviour/Job.h"

class CameraJob : public Job
{
public:
    static CameraJob* newResolutionJob(vector<float> values);
    static CameraJob* newFPSJob(vector<float> values);
    static CameraJob* newSettingsJob(vector<float> values);
    static CameraJob* newAutoExposureJob(vector<float> values);
    static CameraJob* newAutoWhiteBalanceJob(vector<float> values);
    static CameraJob* newAutoGainJob(vector<float> values);
    static CameraJob* newBrightnessJob(vector<float> values); 
    static CameraJob* newContrastJob(vector<float> values);
    static CameraJob* newSaturationJob(vector<float> values);
    static CameraJob* newRedChromaJob(vector<float> values);
    static CameraJob* newBlueChromaJob(vector<float> values);
    static CameraJob* newGainJob(vector<float> values);
    static CameraJob* newExposureJob(vector<float> values);
    static CameraJob* newSelectCameraJob(vector<float> values);
private:
    CameraJob(job_id_t jobid, vector<float> values);
};

#endif

