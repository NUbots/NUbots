/*! @file QSBallisticController.cpp
    @brief A simple ballistic controller
 
    @author Jason Kulk
 
  Copyright (c) 2011 Jason Kulk
 
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

#include "QSBallisticController.h"

#include "QSRelax.h"
#include "QSCatch.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

QSBallisticController::QSBallisticController(const NUData::id_t& joint)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QSBallisticController::QSBallisticController" << endl;
    #endif
    m_relax = new QSRelax(joint, this);
    m_catch = new QSCatch(joint, this);
    
    m_state = m_relax;
}

QSBallisticController::~QSBallisticController()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSBallisticController::~QSBallisticController" << endl;
    #endif
    delete m_relax;
    delete m_catch;
}




