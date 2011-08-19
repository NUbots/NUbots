/*! @file DarwinActionators.h
    @brief Declaration of Bear actionators class
 
    @class DarwinActionators
    @brief The darwin actionators class
 
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

#ifndef DARWINACTIONATORS_H
#define DARWINACTIONATORS_H

#include "NUPlatform/NUActionators.h"
#include "DarwinPlatform.h"


class DarwinActionators : public NUActionators
{
public:
    DarwinActionators(DarwinPlatform*, Robot::CM730*);
    ~DarwinActionators();
    
private:
    void copyToHardwareCommunications();
    void copyToServos();
    void copyToLeds();
    
private:
	Robot::CM730* cm730;
	DarwinPlatform* platform;
	int count;
	//Conversions:
	static const float RATIO_VALUE2RADIAN = 0.001533980; 			//!< 2pi / 4096
	static const float RATIO_RADIAN2VALUE = 651.8986469; 			//!< 4096 / 2pi
	static int Radian2Value(float radian) { return (int)(radian*RATIO_RADIAN2VALUE)+Robot::MX28::CENTER_VALUE; }
	static float Value2Radian(int value) { return (float)(value-Robot::MX28::CENTER_VALUE)*RATIO_VALUE2RADIAN; }

	static vector<string> m_footled_names;
    static unsigned int m_num_footleds;
    static vector<string> m_chestled_names;
    static unsigned int m_num_chestleds;
};

#endif

