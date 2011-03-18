/*! @file QSCatch.cpp
    @brief Implementation of behaviour state class

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

#include "QSCatch.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

QSCatch::QSCatch(const NUData::id_t& joint, const QSBallisticController* parent)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QSCatch::QSCatch" << endl;
    #endif
    m_joint = joint;
    m_parent = parent;
}

QSCatch::~QSCatch()
{
}

void QSCatch::doState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSCatch::doState" << endl;
    #endif
}

BehaviourState* QSCatch::nextState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSCatch::nextState" << endl;
    #endif
    return this;
}

