/*! @file SaveImagesJob.h
    @brief Declaration of SaveImagesJob class.
 
    @class SaveImagesJob
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

#ifndef SAVEIMAGESJOB_H
#define SAVEIMAGESJOB_H

#include "../VisionJob.h"

class SaveImagesJob : public VisionJob
{
public:
    SaveImagesJob(bool saveimages, bool varycamerasettings = false);
    SaveImagesJob(std::istream& input);
    virtual ~SaveImagesJob();
    
    bool saving();
    bool varyCameraSettings();
    
    virtual void summaryTo(std::ostream& output);
    virtual void csvTo(std::ostream& output);
    
    friend std::ostream& operator<<(std::ostream& output, const SaveImagesJob& job);
    friend std::ostream& operator<<(std::ostream& output, const SaveImagesJob* job);
protected:
    virtual void toStream(std::ostream& output) const;
private:
    bool m_save_images;         //!< true if the job is to start saving images, false if the job is to stop saving images
    bool m_vary_settings;       //!< true if the job is to saving images with varying camera settings
};

#endif

