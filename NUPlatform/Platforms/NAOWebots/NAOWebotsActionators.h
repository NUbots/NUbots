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

class NAOWebotsActionators : public NUActionators
{
public:
    NAOWebotsActionators(NAOWebotsPlatform* platform);
    ~NAOWebotsActionators();
    
private:
    void copyToHardwareCommunications();
    
    void getActionatorsFromWebots(NAOWebotsPlatform* platform);
    void enableActionatorsInWebots();
    
    void copyToServos();
    void copyToLeds();
    void copyToTeleporter();
    
private:
    const int m_simulation_step;                    //!< the webots simulation timestep in milliseconds
    
    NAOWebotsPlatform* m_platform;                  //!< a pointer to the platform, in particular platform inherits webots::Robot so use it to access the devices
    
    // Actionators
    static vector<string> m_servo_control_names;    //!< the names of the available joint control methods (usually position and/or torque)
    static vector<string> m_servo_names;            //!< the names of the available joints (eg HeadYaw, AnklePitch etc)
    vector<webots::Servo*> m_servos;                        //!< the actual webots::Servo pointers.
    static vector<string> m_led_names;              //!< the names of the leds available in webots
    vector<webots::LED*> m_leds;                            //!< the actual webots::LED pointers
    static vector<string> m_other_names;            //!< the names of other available actionators in webots
    webots::Emitter* m_teleporter;
};

#endif

