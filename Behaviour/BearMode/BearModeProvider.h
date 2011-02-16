/*! @file BearModeProvider.h
    @brief Declaration of a behaviour provider for the bear
 
    @class BearModeProvider
    @brief A special behaviour for testing bear mode

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

#ifndef BEAR_MODE_PROVIDER_H
#define BEAR_MODE_PROVIDER_H

#include "../BehaviourProvider.h"

#include <vector>
#include <string>

class BearModeProvider : public BehaviourProvider
{
public:
    BearModeProvider(Behaviour* manager);
    ~BearModeProvider();
protected:
    void doBehaviour();
private:
};


#endif

