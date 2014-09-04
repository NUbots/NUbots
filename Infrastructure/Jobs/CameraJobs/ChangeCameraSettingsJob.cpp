/*! @file ChangeCameraSettingsJob.cpp
    @brief Implementation of ChangeCameraSettingsJob class

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

#include "ChangeCameraSettingsJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a ChangeCameraSettingsJob

    @param saveimages true if you want to start saving images, false if you want to stop saving images
 */
ChangeCameraSettingsJob::ChangeCameraSettingsJob(const CameraSettings& settings) : CameraJob(Job::CAMERA_CHANGE_SETTINGS)
{
    m_camera_settings = settings;
}

/*! @brief Constructs a ChangeCameraSettingsJob from stream data
    @param input the stream from which to make a ChangeCameraSettingsJob
 
    Remember that only members introduced at this level are read at this level.
 */
ChangeCameraSettingsJob::ChangeCameraSettingsJob(std::istream& input) : CameraJob(Job::CAMERA_CHANGE_SETTINGS)
{
    m_job_time = 0;

    // read in the m_camera_settings
    input >> m_camera_settings;
}

/*! @brief ChangeCameraSettingsJob destructor
 */
ChangeCameraSettingsJob::~ChangeCameraSettingsJob()
{
}

/*! @brief Gets the camera settings associated with the job
    @param settings this will be updated with a copy of the settings associated with the job
 */
CameraSettings& ChangeCameraSettingsJob::getSettings()
{
    return m_camera_settings;
}

/*! @brief Sets the camera settings associated with the job
    @param newsettings the new camera settigns
 */
void ChangeCameraSettingsJob::setSettings(const CameraSettings& newsettings)
{
    m_camera_settings = newsettings;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void ChangeCameraSettingsJob::summaryTo(std::ostream& output)
{
    output << "ChangeCameraSettingsJob: " << m_job_time << " ";
    output << std::endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void ChangeCameraSettingsJob::csvTo(std::ostream& output)
{
    output << "ChangeCameraSettingsJob, " << m_job_time << ", ";
    output << std::endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void ChangeCameraSettingsJob::toStream(std::ostream& output) const
{
    debug << "ChangeCameraSettingsJob::toStream" << std::endl;
    Job::toStream(output);                  // This writes data introduced at the base level
    CameraJob::toStream(output);            // This writes data introduced at the camera level
                                            // Then we write ChangeCameraSettingsJob specific data
    output << m_camera_settings;
}

/*! @relates ChangeCameraSettingsJob
    @brief Stream insertion operator for a ChangeCameraSettingsJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const ChangeCameraSettingsJob& job)
{
    debug << "<<ChangeCameraSettingsJob" << std::endl;
    job.toStream(output);
    return output;
}

/*! @relates ChangeCameraSettingsJob
    @brief Stream insertion operator for a pointer to ChangeCameraSettingsJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const ChangeCameraSettingsJob* job)
{
    debug << "<<ChangeCameraSettingsJob" << std::endl;
    if (job != NULL)
        job->toStream(output);
    else
        output << "NULL";
    return output;
}
