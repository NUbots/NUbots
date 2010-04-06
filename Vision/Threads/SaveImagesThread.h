/*! @file SaveImagesThread.h
    @brief Declaration of a simple low priority thread for saving images.

    @class SaveImagesThread
    @brief A simple thread to save images when requested
 
    @author Jason Kulk
 
  Copyright (c) 2010 Jason Kulk
 
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SAVEIMAGES_THREAD_H
#define SAVEIMAGES_THREAD_H

#include "Tools/Threading/ConditionalThread.h"

class Vision;

/*! @brief The top-level class
 */
class SaveImagesThread : public ConditionalThread
{
public:
    SaveImagesThread(Vision* vision);
    ~SaveImagesThread();
protected:
    void run();
    
private:
    Vision* m_vision;
};

#endif

