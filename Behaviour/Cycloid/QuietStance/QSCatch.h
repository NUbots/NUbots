/*! @file QSCatch.h
    @brief Declaration of the catching state in the QSBallisticController
 
    @class QSCatch
    @brief The catchin state in the QSBallisticController

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

#ifndef QSCatch_H
#define QSCatch_H

#include "Behaviour/BehaviourState.h"
#include "Infrastructure/NUData.h"
class QSBallisticController;

class QSCatch : public BehaviourState
{
public:
    QSCatch(const NUData::id_t& joint, const QSBallisticController* parent);
    ~QSCatch();
protected:
    void doState();
    BehaviourState* nextState();
private:
    NUData::id_t m_joint;
    const QSBallisticController* m_parent;
};


#endif

