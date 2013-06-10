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

class DarwinJointMapping;
class DarwinPlatform;
namespace Robot
{
    class CM730;
    // class SensorReadManager;
}

/*! 
    @brief Constructs a nubot actionator class with a Darwin backend
        The Darwin backend takes aspects from both the NAO and the Robotis
        backends; the Darwin has a secondary board to perform all the
        communication with hardware, like the NAO. However, it also uses
        Robotis motors, like the Cycloid/Bear.

        This backend is also the most recent, and probably should serve as a
        template for future platforms.
 */ 
class DarwinActionators : public NUActionators
{
public:
    DarwinActionators(DarwinPlatform*, Robot::CM730*);
    ~DarwinActionators();
    
protected:
    void InitialiseMotors();
    void copyToHardwareCommunications();
    void copyToServos();
    void copyToLeds();

    Robot::CM730* cm730;
    DarwinPlatform* platform;
    int count;
    DarwinJointMapping* m_joint_mapping;

    //! Manages sensor read descriptors
    // Robot::SensorReadManager* sensor_read_manager_;

    static vector<string> m_footled_names;
    static unsigned int m_num_footleds;
    static vector<string> m_chestled_names;
    static unsigned int m_num_chestleds;
};

#endif

