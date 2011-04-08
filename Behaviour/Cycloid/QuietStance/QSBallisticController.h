/*! @file QSBallisticController.h
    @brief A simple ballistic controller

        .-- Ballistic Controller ----------.
        |    .---------.      .---------.  |
        |    |  Relax  | ---> |  Catch  |  |
        |    .---------.      .---------.  |
        |          ^               |       |
        |          |               |       |
        |          .---------------.       |
        |    (on catch-action completion)  |
        .----------------------------------.
 
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

#ifndef QSBallisticController_H
#define QSBallisticController_H

#include "Behaviour/BehaviourFSMState.h"
#include "Infrastructure/NUData.h"

class QSRelax;
class QSCatch;

class QSBallisticController : public BehaviourFSMState
{
public:
    QSBallisticController(const NUData::id_t& joint);
    ~QSBallisticController();
private:
    QSRelax* m_relax;
    QSCatch* m_catch;
};

#endif

