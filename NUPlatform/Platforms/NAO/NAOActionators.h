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
#define ALIAS_POSITION "NUP"
#define ALIAS_STIFFNESS "NUS"
#define ALIAS_POSITION_AND_STIFFNESS "NUPS"
#define ALIAS_LED "NUL"
#define ALIAS_ALL "NUA"
public:
    NAOActionators();
    ~NAOActionators();
private:
    void getActionatorsFromAldebaran();
    void startUltrasonics();
    void createALDCMCommands();
    void createALDCMCommand(const char* p_name, ALValue& p_command, unsigned int numactionators);
    void copyToHardwareCommunications();
    
private:
    static std::vector<std::string> m_servo_position_names;
    static unsigned int m_num_servo_positions;
    static std::vector<std::string> m_servo_stiffness_names;
    static unsigned int m_num_servo_stiffnesses;
    
    static std::vector<std::string> m_earled_names;
    static unsigned int m_num_earleds;
    static std::vector<std::string> m_eyeled_names;
    static unsigned int m_num_eyeleds;
    static std::vector<std::string> m_chestled_names;
    static unsigned int m_num_chestleds;
    static std::vector<std::string> m_footled_names;
    static unsigned int m_num_footleds;
    static std::vector<std::string> m_led_names;
    static unsigned int m_num_leds;
    
    static std::vector<std::string> m_other_names;
    static unsigned int m_num_others;
    
    static std::vector<std::string> m_actionator_names;
    static unsigned int m_num_actionators;
    
    DCMProxy* m_al_dcm;
    double m_al_time_offset;
    ALValue m_position_command, m_stiffness_command, m_positionstiffness_command, m_led_command, m_actionator_command;
};

#endif

