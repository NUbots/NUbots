/*! @file SoundJob.h
    @brief Declaration of SoundJob class.
 
    @class SoundJob
    @brief A job to play a sound. We only need one sound job; it plays a specifed file.
 
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

#ifndef SOUNDJOB_H
#define SOUNDJOB_H

#include <string>

#include "Job.h"

class SoundJob : public Job
{
public:
    SoundJob(double time, const std::string filename);
    SoundJob(double time, istream& input);
    virtual ~SoundJob();
    
    std::string getFilename();
    
    virtual void summaryTo(ostream& output);
    virtual void csvTo(ostream& output);
    
    friend ostream& operator<<(ostream& output, const SoundJob& job);
    friend ostream& operator<<(ostream& output, const SoundJob* job);
protected:
    virtual void toStream(ostream& output) const;
private:
    std::string m_filename;         //!< filename of the audio file to be played.
};

#endif

