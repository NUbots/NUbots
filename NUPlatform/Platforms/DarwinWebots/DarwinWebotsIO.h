/*! @file DarwinWebotsIO.h
    @brief Declaration of darwinio class.
 
    @class DarwinWebotsIO
    @brief DarwinWebotsIO class for input and output to streams, files and networks on the Darwin in Webots platform

    @author Jason Kulk, Jed Rietveld
 
  Copyright (c) 2012 Jed Rietveld
 
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

#ifndef DARWINWEBOTSIO_H
#define DARWINWEBOTSIO_H

#include "NUPlatform/NUIO.h"

class NUbot;
class DarwinWebotsPlatform;
class DarwinWebotsNetworkThread;

class DarwinWebotsIO: public NUIO
{
// Functions:
public:
    DarwinWebotsIO(NUbot* nubot, DarwinWebotsPlatform* platform);
    ~DarwinWebotsIO();
    
protected:
private:
    
// Members:
public:
protected:
private:
    DarwinWebotsNetworkThread* m_network;      //!< the simulated network
    
};

#endif

