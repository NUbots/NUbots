/*! @file SaveImagesJob.cpp
    @brief Implementation of SaveImagesJob class

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

#include "SaveImagesJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a SaveImagesJob

    @param saveimages true if you want to start saving images, false if you want to stop saving images
 */
SaveImagesJob::SaveImagesJob(bool saveimages, bool varycamerasettings) : VisionJob(Job::VISION_SAVE_IMAGES)
{
    m_save_images = saveimages;
    m_vary_settings = varycamerasettings;
}

/*! @brief Constructs a SaveImagesJob from stream data
    @param input the stream from which to make a SaveImagesJob
 
    Remember that only members introduced at this level are read at this level.
 */
SaveImagesJob::SaveImagesJob(std::istream& input) : VisionJob(Job::VISION_SAVE_IMAGES)
{
    m_job_time = 0;
    // Temporary read buffers
    bool boolbuffer;

    // read in the m_save_images bool
    input.read(reinterpret_cast<char*>(&boolbuffer), sizeof(boolbuffer));
    m_save_images = boolbuffer;
    input.read(reinterpret_cast<char*>(&boolbuffer), sizeof(boolbuffer));
    m_vary_settings = boolbuffer;
}

/*! @brief SaveImagesJob destructor
 */
SaveImagesJob::~SaveImagesJob()
{
}

/*! @brief Returns true if the job is to start saving, false if the job is to stop saving
 */
bool SaveImagesJob::saving()
{
    return m_save_images;
}

/*! @brief Returns true if the job is to also vary/rotate the camera settings
 */
bool SaveImagesJob::varyCameraSettings()
{
    return m_vary_settings;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void SaveImagesJob::summaryTo(std::ostream& output)
{
    output << "SaveImagesJob: " << m_job_time << " ";
    if (m_save_images == true)
        output << "true";
    else
        output << "false";
    if (m_vary_settings == true)
        output << "true";
    else
        output << "false";
    output << std::endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void SaveImagesJob::csvTo(std::ostream& output)
{
    output << "SaveImagesJob, " << m_job_time << ", ";
    if (m_save_images == true)
        output << "true, ";
    else
        output << "false, ";
    if (m_vary_settings == true)
        output << "true, ";
    else
        output << "false, ";
    output << std::endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void SaveImagesJob::toStream(std::ostream& output) const
{
    Job::toStream(output);                  // This writes data introduced at the base level
    VisionJob::toStream(output);            // This writes data introduced at the vision level
                                            // Then we write SaveImagesJob specific data
    output.write((char*) &m_save_images, sizeof(m_save_images));
    output.write((char*) &m_vary_settings, sizeof(m_vary_settings));
}

/*! @relates SaveImagesJob
    @brief Stream insertion operator for a SaveImagesJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const SaveImagesJob& job)
{
    job.toStream(output);
    return output;
}

/*! @relates SaveImagesJob
    @brief Stream insertion operator for a pointer to SaveImagesJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
std::ostream& operator<<(std::ostream& output, const SaveImagesJob* job)
{
    if (job != NULL)
        job->toStream(output);
    return output;
}
