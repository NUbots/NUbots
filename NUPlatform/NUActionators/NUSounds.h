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
using namespace std;

class NUSounds 
{
public:
    static string PRE_INITIAL;
    static string INITIAL;
    static string READY;
    static string SET;
    static string PLAYING;
    static string FINISHED;
    static string PENALISED;
    static string REQUEST_PICK_UP;
    static string SUBSTITUTE;
    static string RED_TEAM;
    static string BLUE_TEAM;
    static string NUBOT1;
    static string NUBOT2;
    static string NUBOT3;
    static string NUBOT4;
    static string NUBOT5;
    static string NUBOT6;
    static string NUBOT7;
    static string NUBOT8;
    static string NUBOT9;
    static string NUBOT10;
    static string MY_IP_IS;
    
    static string ILLEGAL_INSTRUCTION;
    static string SEG_FAULT;
    static string BUS_ERROR;
    static string ABORT;
    static string UNHANDLED_EXCEPTION;
    static string LOW_BATTERY;
    
    static string START_SAVING_IMAGES;
    static string STOP_SAVING_IMAGES;
    static string LOADING_LUT;
};

#endif

