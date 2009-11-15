/*! @file LightJob.cpp
    @brief Implementation of light job class

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

#include "LightJob.h"

/*! @brief Set the left eye's colour
    @param time the time at which the colour will take place (this will be interpolated by the underlying driver)
    @param colour the (red, green, blue) tuple for the light
 */
LightJob* LightJob::newLEyeJob(float time, vector<float> colour)
{
    LightJob* job = new LightJob(L_EYE, time, colour);
}

/*! @brief Set the right eye's colour
 @param time the time at which the colour will take place (this will be interpolated by the underlying driver)
 @param colour the (red, green, blue) tuple for the light
 */
LightJob* LightJob::newREyeJob(float time, vector<float> colour)
{
    LightJob* job = new LightJob(R_EYE, time, colour);
}

/*! @brief Set the left ear's colour
 @param time the time at which the colour will take place (this will be interpolated by the underlying driver)
 @param colour the (red, green, blue) tuple for the light
 */
LightJob* LightJob::newLEarJob(float time, vector<float> colour)
{
    LightJob* job = new LightJob(L_EAR, time, colour);
}

/*! @brief Set the right ear's colour
 @param time the time at which the colour will take place (this will be interpolated by the underlying driver)
 @param colour the (red, green, blue) tuple for the light
 */
LightJob* LightJob::newREarJob(float time, vector<float> colour)
{
    LightJob* job = new LightJob(R_EAR, time, colour);
}

/*! @brief Set the chest's colour
 @param time the time at which the colour will take place (this will be interpolated by the underlying driver)
 @param colour the (red, green, blue) tuple for the light
 */
LightJob* LightJob::newChestJob(float time, vector<float> colour)
{
    LightJob* job = new LightJob(CHEST, time, colour);
}

/*! @brief Set the left foot's colour
 @param time the time at which the colour will take place (this will be interpolated by the underlying driver)
 @param colour the (red, green, blue) tuple for the light
 */
LightJob* LightJob::newLFootJob(float time, vector<float> colour)
{
    LightJob* job = new LightJob(L_FOOT, time, colour);
}

/*! @brief Set the right foot's colour
 @param time the time at which the colour will take place (this will be interpolated by the underlying driver)
 @param colour the (red, green, blue) tuple for the light
 */
LightJob* LightJob::newRFootJob(float time, vector<float> colour)
{
    LightJob* job = new LightJob(R_FOOT, time, colour);
}

/*! A private constructor
 */
LightJob::LightJob(job_id_t jobid, float time, vector<float> colour)
{
    m_job_type = LIGHT;
    m_job_id = jobid;
    m_job_time = time;
    m_values = colour;
}
