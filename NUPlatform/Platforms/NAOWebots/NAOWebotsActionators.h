/*! @file NAOWebotsActionators.h
    @brief Declaration of NAO in Webots actionators class

    @author Jason Kulk
 
    @class NAOWebotsActionators
    @brief A NAO in Webots actionators
 
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

#ifndef NAOWEBOTSACTIONATORS_H
#define NAOWEBOTSACTIONATORS_H

#include "NUPlatform/NUActionators.h"
#include "NAOWebotsPlatform.h"

#include "webots/Robot.hpp"
using namespace webots;

class NAOWebotsActionators : public NUActionators
{
public:
    NAOWebotsActionators(NAOWebotsPlatform* platform);
    ~NAOWebotsActionators();
    
private:
    void copyToHardwareCommunications();
    
    void getActionatorsFromWebots(NAOWebotsPlatform* platform);
    void enableActionatorsInWebots();
private:
    static const int m_simulation_step = 40;
    
    NAOWebotsPlatform* m_platform;
    
    // Actionators
    static vector<string> m_servo_control_names;    //<! the names of the available joint control methods (usually position and/or torque)
    static vector<string> m_servo_names;
    vector<Servo*> m_servos;
    Servo* m_camera_control;                        //!< a servo which selects which camera to use, webots doesn't have any 'camera setttings' as such so this is the only camera control
    static vector<string> m_led_names;
    vector<LED*> m_leds;
    //! @todo TODO: add the sound actionator
};

#endif

