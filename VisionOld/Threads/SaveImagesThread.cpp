/*! @file SaveImagesThread.h
    @brief Implementation of the saveimages thread class.

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

#include "SaveImagesThread.h"
#include "VisionOld/Vision.h"

#include "debug.h"
#include "debugverbosityvision.h"

#include <errno.h>

/*! @brief Constructs the network thread
 */

SaveImagesThread::SaveImagesThread(Vision* vision) : ConditionalThread(string("SaveImagesThread"), 0)
{
    #if DEBUG_VISION_VERBOSITY > 0
        debug << "SaveImagesThread::SaveImagesThread(" << vision << ") with priority " << static_cast<int>(m_priority) << endl;
    #endif
    m_vision = vision;
    start();
}

SaveImagesThread::~SaveImagesThread()
{
    #if DEBUG_VISION_VERBOSITY > 0
        debug << "SaveImagesThread::~SaveImagesThread()" << endl;
    #endif
}

/*! @brief The save images main loop
 
 */
void SaveImagesThread::run()
{
    #if DEBUG_VISION_VERBOSITY > 0
        debug << "SaveImagesThread::run()" << endl;
    #endif
    
    int err = 0;
    while (err == 0 && errno != EINTR)
    {
        wait();
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        m_vision->SaveAnImage();
        // -----------------------------------------------------------------------------------------------------------------------------------------------------------------
        
    } 
    errorlog << "SaveImagesThread is exiting. err: " << err << " errno: " << errno << endl;
}
