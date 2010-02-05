/*! @file NAOActionators.h
    @brief Declaration of NAO actionators class

    @author Jason Kulk
 
    @class NAOActionators
    @brief A NAO actionators
 
    @author Jason Kulk
 
  Copyright (c) 2009 Jason Kulk
 
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

#ifndef NAOACTIONATORS_H
#define NAOACTIONATORS_H

#include "NUPlatform/NUActionators.h"
#include "NUNAO.h"

class NAOActionators : public NUActionators
{
#define ALIAS_POSITION "PositionActionators"
#define ALIAS_STIFFNESS "StiffnessActionators"
#define ALIAS_LED "LedActionators"
public:
    NAOActionators();
    ~NAOActionators();
private:
    void getActionatorsFromALDCM();
    void copyToHardwareCommunications();
    
private:
    static vector<string> m_servo_control_names;
    static vector<string> m_servo_position_names;
    static vector<string> m_servo_stiffness_names;
    static vector<string> m_led_names;
    static vector<string> m_actionator_names;
    
    DCMProxy* m_al_dcm;
    double m_al_time_offset;
};

#endif

