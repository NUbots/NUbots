/*! @file JWalkPush.h
    @brief Declaration of a jwalk's push state
 
    @class JWalkPush
    @brief A jwalk push state
 
    @author Jason Kulk
 
  Copyright (c) 2010, 2011 Jason Kulk
 
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

#ifndef JWALKPUSH_H
#define JWALKPUSH_H

#include "JWalkState.h"

class JWalkPush : public JWalkState
{
public:
    JWalkPush(const NUData::id_t& leg);
    ~JWalkPush();
    void doIt();
    JWalkPush* next();
private:
};

#endif

