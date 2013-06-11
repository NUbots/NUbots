/*! @file NUSounds.h
    @brief Declaration of a sounds.
 
    @class NUSounds
    @brief Definitions of sound filenames
 
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

#ifndef NUSOUNDS_H
#define NUSOUNDS_H

#include <string>


class NUSounds 
{
public:
    static std::string PRE_INITIAL;
    static std::string INITIAL;
    static std::string READY;
    static std::string SET;
    static std::string PLAYING;
    static std::string FINISHED;
    static std::string PENALISED;
    static std::string REQUEST_PICK_UP;
    static std::string SUBSTITUTE;
    static std::string RED_TEAM;
    static std::string BLUE_TEAM;
    static std::string NUBOT1;
    static std::string NUBOT2;
    static std::string NUBOT3;
    static std::string NUBOT4;
    static std::string NUBOT5;
    static std::string NUBOT6;
    static std::string NUBOT7;
    static std::string NUBOT8;
    static std::string NUBOT9;
    static std::string NUBOT10;
    static std::string MY_IP_IS;
    
    static std::string ILLEGAL_INSTRUCTION;
    static std::string SEG_FAULT;
    static std::string BUS_ERROR;
    static std::string ABORT;
    static std::string UNHANDLED_EXCEPTION;
    static std::string LOW_BATTERY;
    
    static std::string START_SAVING_IMAGES;
    static std::string STOP_SAVING_IMAGES;
    static std::string LOADING_LUT;
};

#endif

