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
    static joint_id_t TorsoYaw;
    static joint_id_t TorsoPitch;
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
        Body,
        All
    };
public:
    NUActionatorsData();
    ~NUActionatorsData();
    
    void removeCompletedActions(double currenttime);
    
    // Get methods for joints
    bool getJointPositions(double& time, vector<bool>& isvalid, vector<float>& positions, vector<float>& gains);
    bool getJointVelocities(double& time, vector<bool>& isvalid, vector<float>& velocities, vector<float>& gains);
    bool getJointTorques(double& time, vector<bool>& isvalid, vector<float>& torques, vector<float>& gains);
    
    // Set methods for a joint without specifying a gain
    bool setJointPosition(joint_id_t jointid, double time, float position);
    bool setJointVelocity(joint_id_t jointid, double time, float velocity);
    bool setJointStiffness(joint_id_t jointid, double time, float stiffness);
    bool setJointTorque(joint_id_t jointid, double time, float torque);
    
    // Set methods for a joint
    bool setJointPosition(joint_id_t jointid, double time, float position, float gain);
    bool setJointVelocity(joint_id_t jointid, double time, float velocity, float gain);
    bool setJointTorque(joint_id_t jointid, double time, float torque, float gain);
    
    // Set methods for the joints without specifying a gain
    bool setJointPositions(bodypart_id_t partid, double time, const vector<float>& positions);
    bool setJointVelocities(bodypart_id_t partid, double time, const vector<float>& velocities);
    bool setJointStiffnesses(bodypart_id_t partid, double time, const vector<float>& stiffnesses);
    bool setJointTorques(bodypart_id_t partid, double time, const vector<float>& torques);
    
    bool setJointPositions(bodypart_id_t partid, double time, const vector<float>& positions, const vector<float>& gain);
    bool setJointVelocities(bodypart_id_t partid, double time, const vector<float>& velocities, const vector<float>& gain);
    bool setJointTorques(bodypart_id_t partid, double time, const vector<float>& torques, const vector<float>& gain);

    void setAvailableJoints(const vector<string>& joints);
    void setAvailableLeds(const vector<string>& leds);
    void setAvailableActionators(const vector<string>& actionators);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUActionatorsData& p_sensor);
    friend istream& operator>> (istream& input, NUActionatorsData& p_sensor);
    
private:
    void addActionator(actionator_t** p_actionator, string actionatorname, actionator_t::actionator_id_t actionatorid);
    
    bool getJointData(actionator_t* p_actionator, double& time, vector<bool>& isvalid, vector<float>& data, vector<float>& gains);
    
    bool setJointData(actionator_t* p_actionator, joint_id_t jointid, double time, float data);
    bool setJointData(actionator_t* p_actionator, joint_id_t jointid, double time, float data, float gain);

    void expandGain(const vector<joint_id_t>& ids, const vector<float>& gains, vector<bool>& isgainvalid, vector<float>& allgains);
    void expandData(const vector<joint_id_t>& ids, const vector<float>& data, vector<bool>& isdatavalid, vector<float>& alldata);
    void expandAction(const vector<joint_id_t>& ids, const vector<float>& data, const vector<float>& gains, vector<bool>& isvalid, vector<float>& data, vector<float>& gains);

    void addAction(actionator_t* p_actionator, double time, const vector<bool>& isvalid, const vector<float>& alldata, const vector<float>& allgains);
    void addAction(actionator_t* p_actionator, double time, const vector<bool>& isdatavalid, const vector<float>& alldata, const vector<bool>& isgainvalid, const vector<float>& allgains);
    
    bool setJointsGain(actionator_t* p_actionator, bodypart_id_t partid, double time, const vector<float>& gains);
    bool setJointsData(actionator_t* p_actionator, bodypart_id_t partid, double time, const vector<float>& data);
    bool setJointsData(actionator_t* p_actionator, bodypart_id_t partid, double time, const vector<float>& data, const vector<float>& gains);
    
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
    vector<joint_id_t> m_body_ids;
    vector<joint_id_t> m_all_joint_ids;
    int m_num_head_joints;
    int m_num_arm_joints;
    int m_num_torso_joints;
    int m_num_leg_joints;
    int m_num_body_joints;
    int m_num_joints;
};

#endif

