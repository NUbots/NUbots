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

#include "nubotconfig.h"
#include <dcmproxy.h>

class NAOActionators : public NUActionators
{
#define ALIAS_POSITION "PositionActionators"
#define ALIAS_STIFFNESS "StiffnessActionators"
#define ALIAS_LED "LedActionators"
public:
    NAOActionators();
    ~NAOActionators();
private:
    void getActionatorsFromAldebaran();
    void createALDCMCommands();
    void createALDCMCommand(const char* p_name, ALValue& p_command, unsigned int numactionators);
    void copyToHardwareCommunications();
    
private:
    static vector<string> m_servo_control_names;
    static vector<string> m_servo_names;
    static vector<string> m_servo_position_names;
    static unsigned int m_num_servo_positions;
    static vector<string> m_servo_stiffness_names;
    static unsigned int m_num_servo_stiffnesses;
    static vector<string> m_led_names;
    static unsigned int m_num_leds;
    static vector<string> m_actionator_names;
    static unsigned int m_num_actionators;
    
    DCMProxy* m_al_dcm;
    double m_al_time_offset;
    ALValue m_position_command, m_stiffness_command, m_led_command;
};

#endif

