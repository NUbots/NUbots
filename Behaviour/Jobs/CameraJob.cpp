/*! @file CameraJob.cpp
    @brief Implementation of camera job class

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

#include "CameraJob.h"


/*! @brief Change the camera's resolution
    @param values the desired resolution (width, height)
 */
CameraJob* CameraJob::newResolutionJob(vector<float> values)
{
    CameraJob* job = new CameraJob(RESOLUTION, values);
}

/*! @brief Change the camera's frame rate
 @param values the new frame rate
 */
CameraJob* CameraJob::newFPSJob(vector<float> values)
{
    CameraJob* job = new CameraJob(FPS, values);
}

/*! @brief Apply new camera settings to the currently selected camera
 
    Use this function to change all of the camera settings at once. Because the settings are in a vector, you will
    need to be careful with the order. Alternatively, use any of the more specific functions.
 
    @param values the camera settings (auto_exposure, auto_wb, auto_gain, brightness, contrast, saturation, red_chr, blue_chr, gain, exposure)
 */
CameraJob* CameraJob::newSettingsJob(vector<float> values)
{
    CameraJob* job = new CameraJob(SETTINGS, values);
}

/*! @brief Turn the camera's auto exposure on and off
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newAutoExposureJob(vector<float> values)
{
    CameraJob* job = new CameraJob(AUTO_EXPOSURE, values);
}

/*! @brief Turn the camera's auto white balance on and off
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newAutoWhiteBalanceJob(vector<float> values)
{
    CameraJob* job = new CameraJob(AUTO_WHITE_BALANCE, values);
}

/*! @brief Turn the camera's auto gain on and off
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newAutoGainJob(vector<float> values)
{
    CameraJob* job = new CameraJob(AUTO_GAIN, values);
}

/*! @brief Change the camera's brightness
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newBrightnessJob(vector<float> values)
{
    CameraJob* job = new CameraJob(BRIGHTNESS, values);
}

/*! @brief Change the camera's contrast
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newContrastJob(vector<float> values)
{
    CameraJob* job = new CameraJob(CONTRAST, values);
}

/*! @brief Change the camera's saturation
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newSaturationJob(vector<float> values)
{
    CameraJob* job = new CameraJob(SATURATION, values);
}

/*! @brief Change the camera's red chroma
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newRedChromaJob(vector<float> values)
{
    CameraJob* job = new CameraJob(RED_CHROMA, values);
}

/*! @brief Change the camera's blue chroma
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newBlueChromaJob(vector<float> values)
{
    CameraJob* job = new CameraJob(BLUE_CHROMA, values);
}

/*! @brief Change the camera's gain
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newGainJob(vector<float> values)
{
    CameraJob* job = new CameraJob(GAIN, values);
}

/*! @brief Change the camera's exposure
 @param values the values associated with the camera setting
 */
CameraJob* CameraJob::newExposureJob(vector<float> values)
{
    CameraJob* job = new CameraJob(EXPOSURE, values);
}

/*! @brief Select which camera(s) should be active
 @param values the ids of the cameras to be selected
 */
CameraJob* CameraJob::newSelectCameraJob(vector<float> values)
{
    CameraJob* job = new CameraJob(SELECT_CAMERA, values);
}

/*! A private constructor so that behaviour does not need to know about the job ids
 */
CameraJob::CameraJob(job_id_t jobid, vector<float> values)
{
    m_job_type = CAMERA;
    m_job_id = jobid;
    m_values = values;
}
