/*! @file NUActionatorsData.h
    @brief Declaration of a actionator data class to store actionator data in a platform independent way
    @author Jason Kulk
 
    @class NUActionatorsData
    @brief A actionator class to store actionator data in a platform independent way
 
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

#ifndef NUACTIONATORSDATA_H
#define NUACTIONATORSDATA_H

#include "actionator_t.h"

#include <vector>
#include <string>
using namespace std;

class NUActionatorsData
{
public:
    typedef int joint_id_t;
    typedef int led_id_t;
    
    static const int ACTIONATOR_MISSING = -1;
    // joints
    static joint_id_t HeadYaw;
    static joint_id_t HeadPitch;
    static joint_id_t LShoulderPitch;
    static joint_id_t LShoulderRoll;
    static joint_id_t LElbowYaw;
    static joint_id_t LElbowRoll;
    static joint_id_t RShoulderPitch;
    static joint_id_t RShoulderRoll;
    static joint_id_t RElbowYaw;
    static joint_id_t RElbowRoll;
    static joint_id_t LHipYaw;
    static joint_id_t LHipYawPitch;
    static joint_id_t LHipPitch;
    static joint_id_t LHipRoll;
    static joint_id_t LKneePitch;
    static joint_id_t LAnklePitch;
    static joint_id_t LAnkleRoll;
    static joint_id_t RHipYaw;
    static joint_id_t RHipYawPitch;
    static joint_id_t RHipPitch;
    static joint_id_t RHipRoll;
    static joint_id_t RKneePitch;
    static joint_id_t RAnklePitch;
    static joint_id_t RAnkleRoll;
    // leds
    static led_id_t LEar;
    static led_id_t REar;
    static led_id_t LEye;
    static led_id_t REye;
    static led_id_t Chest;
    static led_id_t LFoot;
    static led_id_t RFoot;
    enum bodypart_id_t
    {
        Head,
        LeftArm,
        RightArm,
        Torso,
        LeftLeg,
        RightLeg,
        All
    };
public:
    NUActionatorsData();
    ~NUActionatorsData();

    void setAvailableJoints(const vector<string>& joints);
    void setAvailableLeds(const vector<string>& leds);
    void setAvailableActionators(const vector<string>& actionators);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUActionatorsData& p_sensor);
    friend istream& operator>> (istream& input, NUActionatorsData& p_sensor);
    
private:
    void addActionator(actionator_t** p_actionator, string actionatorname, actionator_t::actionator_id_t actionatorid);
    
public:
    // NAMED ACTIONATORS
    actionator_t* JointPositions;           //!< The joint position actuators; available on all robotic platforms
    actionator_t* JointVelocities;          //!< The joint velocity actuators; available on all robotic platforms
    actionator_t* JointTorques;             //!< The joint torque actuators; avaliable only in Webots 
    actionator_t* CameraControl;            //!< The camera control actionator, use this actionator to swap cameras and change settings
    actionator_t* Leds;                     //!< The leds
    actionator_t* Sound;                    //!< The sound actionator
    
private:
    vector<actionator_t*> m_actionators;
    vector<joint_id_t> m_head_ids;
    vector<joint_id_t> m_larm_ids;
    vector<joint_id_t> m_rarm_ids;
    vector<joint_id_t> m_torso_ids;
    vector<joint_id_t> m_lleg_ids;
    vector<joint_id_t> m_rleg_ids;
};

#endif

