/*! @file QuietStanceProvider.h
    @brief Declaration of a ballistic controller for quasi-quiet stance
 
    @class QuietStanceProvider
    @brief A special behaviour for testing the ballistic controller for quasi-quiet stance
 
    The architecture of the behaviour is a little different. Each joint (lankle, rankle, lhip, rhip) has an independent
    state machine. 
 
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

#ifndef QUIETSTANCE_PROVIDER_H
#define QUIETSTANCE_PROVIDER_H

#include "../../BehaviourProvider.h"
class QSBallisticController;

class QuietStanceProvider : public BehaviourProvider
{
public:
    QuietStanceProvider(Behaviour* manager);
    ~QuietStanceProvider();
protected:
    void doBehaviour();
private:
    QSBallisticController* m_lankle;
    QSBallisticController* m_rankle;
    QSBallisticController* m_lhip;
    QSBallisticController* m_rhip;
};

#endif

