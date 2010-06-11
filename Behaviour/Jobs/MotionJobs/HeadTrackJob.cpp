/*! @file HeadTrackJob.cpp
    @brief Implementation of HeadTrackJob class

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

#include "HeadTrackJob.h"
#include "Vision/FieldObjects/Object.h"

#include "debug.h"
#include "debugverbosityjobs.h"

/*! @brief Constructs a track visual object job
    @param object the object to be tracked
    @param centreelevation the target elevation in the image for the object to be
    @param centrebearing the target bearing in the image for the object to be
 */
HeadTrackJob::HeadTrackJob(const Object& object, float centreelevation, float centrebearing) : MotionJob(MOTION_TRACK)
{
    if (object.TimeSeen() > 0)
    {
        m_elevation = -object.ScreenYTheta();
        m_bearing = object.ScreenXTheta();
    }
    else
    {
        // use localisation data
        // m_elevation = object.filteredImageElevation();
        // m_bearing = object.filteredImageBearing();
    }
	m_job_time = 0;
    m_centre_elevation = centreelevation;
    m_centre_bearing = centrebearing;
}

/*! @brief Constructs a track visual object job where elevation and bearing specify the position in the image
    @param elevation the elevation in the image
    @param bearing the bearing in the image
    @param centreelevation the target elevation in the image for the object to be
    @param centrebearing the target bearing in the image for the object to be
 */
HeadTrackJob::HeadTrackJob(float elevation, float bearing, float centreelevation, float centrebearing) : MotionJob(MOTION_TRACK)
{
    m_elevation = -elevation;
    m_bearing = bearing;
    m_centre_elevation = centreelevation;
    m_centre_bearing = centrebearing;
	m_job_time = 0;
}

/*! @brief Constructs a HeadTrackJob from stream data
 */
HeadTrackJob::HeadTrackJob(istream& input) : MotionJob(MOTION_TRACK)
{
    m_job_time = 0;

    float floatBuffer;
    input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
    m_elevation = floatBuffer;
    input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
    m_bearing = floatBuffer;
    input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
    m_centre_elevation = floatBuffer;
    input.read(reinterpret_cast<char*>(&floatBuffer), sizeof(floatBuffer));
    m_centre_bearing = floatBuffer;
}

/*! @brief WalkJob destructor
 */
HeadTrackJob::~HeadTrackJob()
{
}

/*! @brief Gets the data associated with the job
    @param elevation
    @param bearing
    @param centreelevation
    @param centrebearing
 */
void HeadTrackJob::getData(float& elevation, float& bearing, float& centreelevation, float& centrebearing)
{
    elevation = m_elevation;
    bearing = m_bearing;
    centreelevation = m_centre_elevation;
    centrebearing = m_centre_bearing;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void HeadTrackJob::summaryTo(ostream& output)
{
    output << "HeadTrackJob: " << m_job_time << " ";
    output << m_elevation << " " << m_bearing << " " << m_centre_elevation << " " << m_centre_bearing << endl;
    output << endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void HeadTrackJob::csvTo(ostream& output)
{
    output << "HeadTrackJob: " << m_job_time << ", ";
    output << m_elevation << ", " << m_bearing << ", " << m_centre_elevation << ", " << m_centre_bearing << endl;
    output << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void HeadTrackJob::toStream(ostream& output) const
{
    #if DEBUG_JOBS_VERBOSITY > 1
        debug << "HeadTrackJob::toStream" << endl;
    #endif
    Job::toStream(output);                  // This writes data introduced at the base level
    MotionJob::toStream(output);            // This writes data introduced at the motion level
    // Then we write HeadTrackJob specific data (m_times and m_head_positions)
    output.write((char*) &m_elevation, sizeof(m_elevation));
    output.write((char*) &m_bearing, sizeof(m_bearing));
    output.write((char*) &m_centre_elevation, sizeof(m_centre_elevation));
    output.write((char*) &m_centre_bearing, sizeof(m_centre_bearing));
}

/*! @relates HeadTrackJob
    @brief Stream insertion operator for a HeadTrackJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadTrackJob& job)
{
    #if DEBUG_JOBS_VERBOSITY > 0
        debug << "<<HeadTrackJob" << endl;
    #endif
    job.toStream(output);
    return output;
}

/*! @relates HeadTrackJob
    @brief Stream insertion operator for a pointer to HeadTrackJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const HeadTrackJob* job)
{
    #if DEBUG_JOBS_VERBOSITY > 0
        debug << "<<HeadTrackJob" << endl;
    #endif
    if (job != NULL)
        job->toStream(output);
    return output;
}
