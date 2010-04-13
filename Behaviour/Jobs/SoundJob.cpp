/*! @file SoundJob.cpp
    @brief Implementation of SoundJob class

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

#include "SoundJob.h"
#include "debug.h"
#include "debugverbosityjobs.h"

using namespace std;

/*! @brief Constructs a SoundJob
    @param time the time the sound is to be played
    @param filename filename of the sound to be played
 */
SoundJob::SoundJob(double time, const std::string filename) : Job(Job::SOUND, Job::SOUND_FILE)
{
    m_job_time = time;
    m_filename = filename;
}

/*! @brief Constructs a SoundJob from stream data
    @param time the time the sound is to be played (this is passed down from the base Job level)
    @param input the stream from which to make a SoundJob
 
    Remember that only members introduced at this level are read at this level.
 */
SoundJob::SoundJob(double time, istream& input) : Job(Job::SOUND, Job::SOUND_FILE)
{
    m_job_time = time;
    input >> m_filename;
}

/*! @brief SoundJob destructor
 */
SoundJob::~SoundJob()
{
}

/*! @brief Returns the filename of the sound to be played
 */
string SoundJob::getFilename()
{
    return m_filename;
}

/*! @brief Prints a human-readable summary to the stream
 @param output the stream to be written to
 */
void SoundJob::summaryTo(ostream& output)
{
    output << "SoundJob: " << m_job_time << " " << m_filename;
    output << endl;
}

/*! @brief Prints a csv version to the stream
 @param output the stream to be written to
 */
void SoundJob::csvTo(ostream& output)
{
    output << "SoundJob, " << m_job_time << ", " << m_filename;
    output << endl;
}

/*! @brief A helper function to ease writing Job objects to classes
 
    This function calls its parents versions of the toStream, each parent
    writes the members introduced at that level

    @param output the stream to write the job to
 */
void SoundJob::toStream(ostream& output) const
{
    Job::toStream(output);                  // This writes data introduced at the base level
                                            // Then we write SoundJob specific data
    output << m_filename;
}

/*! @relates SoundJob
    @brief Stream insertion operator for a SoundJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const SoundJob& job)
{
    job.toStream(output);
    return output;
}

/*! @relates SoundJob
    @brief Stream insertion operator for a pointer to SoundJob

    @param output the stream to write to
    @param job the job to be written to the stream
 */
ostream& operator<<(ostream& output, const SoundJob* job)
{
    if (job != NULL)
        job->toStream(output);
    else
        output << "NULL";
    return output;
}
