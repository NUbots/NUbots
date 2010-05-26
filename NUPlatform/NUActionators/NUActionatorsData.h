/*! @file NUActionatorsData.h
    @brief Declaration of a actionator data class to store actionator data in a platform independent way
    @author Jason Kulk
 
    @class NUActionatorsData
    @brief A actionator class to store actionator data in a platform independent way
 
    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
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

template <class T>
class actionator_t;

#include <vector>
#include <string>
using namespace std;

class NUActionatorsData
{
public:
    typedef int joint_id_t;
    enum bodypart_id_t
    {
        HeadJoints,
        LeftArmJoints,
        RightArmJoints,
        TorsoJoints,
        LeftLegJoints,
        RightLegJoints,
        BodyJoints,
        AllJoints
    };
    
    enum ledgroup_id_t
    {
        LeftEarLeds,
        RightEarLeds,
        LeftEyeLeds,
        RightEyeLeds,
        ChestLeds,
        LeftFootLeds,
        RightFootLeds,
        AllLeds
    };

    static const int ACTIONATOR_MISSING = -1;
    
    // joints
    static joint_id_t HeadPitch;
    static joint_id_t HeadYaw;
    static joint_id_t LShoulderRoll;
    static joint_id_t LShoulderPitch;
    static joint_id_t LElbowRoll;
    static joint_id_t LElbowYaw;
    static joint_id_t RShoulderRoll;
    static joint_id_t RShoulderPitch;
    static joint_id_t RElbowRoll;
    static joint_id_t RElbowYaw;
    static joint_id_t TorsoRoll;
    static joint_id_t TorsoPitch;
    static joint_id_t TorsoYaw;
    static joint_id_t LHipRoll;
    static joint_id_t LHipPitch;
    static joint_id_t LHipYawPitch;
    static joint_id_t LHipYaw;
    static joint_id_t LKneePitch;
    static joint_id_t LAnkleRoll;
    static joint_id_t LAnklePitch;
    static joint_id_t RHipRoll;
    static joint_id_t RHipPitch;
    static joint_id_t RHipYawPitch;
    static joint_id_t RHipYaw;
    static joint_id_t RKneePitch;
    static joint_id_t RAnkleRoll;
    static joint_id_t RAnklePitch;
    
    double CurrentTime;
public:
    NUActionatorsData();
    ~NUActionatorsData();
    
    // Methods for setting which actionators are available on init
    void setAvailableJointControlMethods(const vector<string>& methodnames);
    void setAvailableJoints(const vector<string>& jointnamess);
    void setAvailableCameraSettings(const vector<string>& camerasettingnames);
    void setAvailableLeds(const vector<string>& lednames);
    void setAvailableOtherActionators(const vector<string>& actionatornames);
    
    void preProcess();
    void postProcess(double currenttime);
    
    // Misc. get methods
    int getNumberOfJoints(bodypart_id_t partid);
    vector<joint_id_t>& getJointIndices(bodypart_id_t partid);
    int getNumberOfLeds(ledgroup_id_t groupid);
    
    // Get methods for NUActionators
        // joint data for the NUActionators
    bool getNextJointPosition(joint_id_t id, double& time, float& position, float& velocity, float& gain);
    bool getLastJointPosition(joint_id_t id, double& time, float& position, float& velocity, float& gain);
    bool getNextJointPositions(vector<bool>& isvalid, vector<double>& time, vector<float>& positions, vector<float>& velocities, vector<float>& gains);
    bool getNextJointTorques(vector<bool>& isvalid, vector<double>& time, vector<float>& torques, vector<float>& gains);
        // other data for the NUActionators
    bool getNextLeds(vector<bool>& isvalid, vector<double>& time, vector<float>& redvalues, vector<float>& greenvalues, vector<float>& bluevalues);
    bool getNextSounds(bool& isvalid, double& time, vector<string>& sounds);
        // magical data for the NUActionators
    bool getNextTeleportation(bool& isvalid, double& time, vector<float>& data);
    
    // Add methods to be used by modules (ie by Vision, Motion etc)
        // add new joint commands
    bool addJointPosition(joint_id_t jointid, double time, float position, float velocity, float gain);
    bool addJointPositions(joint_id_t jointid, const vector<double>& times, const vector<float>& positions, const vector<float>& velocities, float gain);
    bool addJointPositions(joint_id_t jointid, const vector<double>& times, const vector<float>& positions, const vector<float>& velocities, const vector<float>& gains);
    bool addJointTorque(joint_id_t jointid, double time, float torque, float gain);
    bool addJointTorques(joint_id_t jointid, const vector<double>& times, const vector<float>& torques, float gain);
    bool addJointTorques(joint_id_t jointid, const vector<double>& times, const vector<float>& torques, const vector<float>& gains);
    
    bool addJointPositions(bodypart_id_t partid, double time, const vector<float>& positions, const vector<float>& velocities, float gain);
    bool addJointPositions(bodypart_id_t partid, double time, const vector<float>& positions, const vector<float>& velocities, const vector<float>& gains);
    bool addJointTorques(bodypart_id_t partid, double time, const vector<float>& torques, float gain);
    bool addJointTorques(bodypart_id_t partid, double time, const vector<float>& torques, const vector<float>& gains);
    bool addJointPositions(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& positions, const vector<vector<float> >& velocities, float gain);
    bool addJointPositions(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& positions, const vector<vector<float> >& velocities, const vector<float>& gain);
    bool addJointPositions(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& positions, const vector<vector<float> >& velocities, const vector<vector<float> >& gains);
    bool addJointTorques(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& torques, float gain);
    bool addJointTorques(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& torques, const vector<float>& gain);
    bool addJointTorques(bodypart_id_t partid, const vector<vector<double> >& times, const vector<vector<float> >& torques, const vector<vector<float> >& gains);
    
        // add other commands
    bool addLeds(ledgroup_id_t ledgroup, double time, float red, float blue, float green);
    bool addLeds(ledgroup_id_t ledgroup, double time, const vector<float>& values);
    bool addLeds(ledgroup_id_t ledgroup, double time, const vector<vector<float> >& values);
    bool addLeds(ledgroup_id_t ledgroup, const vector<int>& indices, double time, const vector<vector<float> >& values);
    bool addSound(double time, string sound);
    bool addSounds(double time, vector<string> sound);
        // add magic commands
    bool addTeleportation(double time, float x, float y, float bearing);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUActionatorsData& p_sensor);
    friend istream& operator>> (istream& input, NUActionatorsData& p_sensor);
    
private:
    void addJointActionator(string actionatorname);
    void addLedActionator(string actionatorname);
    template <typename T> void addActionator(vector<actionator_t<T>*>& actionatorgroup, string actionatorname, typename actionator_t<T>::actionator_type_t actionatortype);
    template <typename T> void addActionator(actionator_t<T>*& p_actionator, string actionatorname, typename actionator_t<T>::actionator_type_t actionatortype);
    
    vector<joint_id_t>& getSelectedJoints(bodypart_id_t partid);
    vector<int>& getSelectedLeds(ledgroup_id_t groupid);
    
    string simplifyName(const string& input);
    void simplifyNames(const vector<string>& input, vector<string>& output);
    void removeColours(const vector<string>& input, vector<string>& output);
    
private:
    vector<actionator_t<float>*> m_all_actionators;        //!< a vector with every float actionator
    vector<actionator_t<string>*> m_all_string_actionators;      //!< a vector with every string actionator
    // Limb position and torque actionators
    bool m_positionactionation;
    bool m_torqueactionation;
    vector<actionator_t<float>*> PositionActionators;      //!< the actionators to change the position, velocity and postion-gain
    vector<actionator_t<float>*> TorqueActionators;        //!< the actionators to change the torque, and torque-gain

    // Peripheral actionators 
    vector<actionator_t<float>*> LedActionators;           //!< The led actionators
    actionator_t<string>* Sound;                            //!< The sound actionator
    
    // Magic actionators
    actionator_t<float>* Teleporter;                       //!< A magical actionator that teleports the robot from one location to another
    
    // Variables to provide fast access to body part actionator groups
    vector<joint_id_t> m_head_ids;                  //!< a vector of joint_id_t's for each joint in the head
    vector<joint_id_t> m_larm_ids;                  //!< a vector of joint_id_t's for each joint in the left arm
    vector<joint_id_t> m_rarm_ids;                  //!< a vector of joint_id_t's for each joint in the right arm
    vector<joint_id_t> m_torso_ids;                 //!< a vector of joint_id_t's for each joint in the torso
    vector<joint_id_t> m_lleg_ids;                  //!< a vector of joint_id_t's for each joint in the left leg
    vector<joint_id_t> m_rleg_ids;                  //!< a vector of joint_id_t's for each joint in the right leg
    vector<joint_id_t> m_body_ids;                  //!< a vector of joint_id_t's for each joint in the body (ie all except the head)
    vector<joint_id_t> m_all_joint_ids;             //!< a vector of joint_id_t's for every joint
    int m_num_head_joints;                          //!< the number of joints in the head
    int m_num_arm_joints;                           //!< the number of joints in a single arm. Note it assumes both arms are the same 
    int m_num_torso_joints;                         //!< the number of joints in the torso
    int m_num_leg_joints;                           //!< the number of joints in a single leg. Note it assumes both legs are the same
    int m_num_body_joints;                          //!< the number of joints in the body
    int m_num_joints;                               //!< the total number of joints int the robot
    
    // Variables to provide fast access to groups of leds
    vector<int> m_lear_ids;                         //!< a vector of indicies into LedActionators, where each index is a member of the left ear led group
    vector<int> m_rear_ids;                         //!< a vector of indicies into LedActionators, where each index is a member of the right ear led group                         
    vector<int> m_leye_ids;                         //!< a vector of indicies into LedActionators, where each index is a member of the left eye led group
    vector<int> m_reye_ids;                         //!< a vector of indicies into LedActionators, where each index is a member of the right eye led group
    vector<int> m_chest_ids;                        //!< a vector of indicies into LedActionators, where each index is a member of the chest led group
    vector<int> m_lfoot_ids;                        //!< a vector of indicies into LedActionators, where each index is a member of the left foot led group
    vector<int> m_rfoot_ids;                        //!< a vector of indicies into LedActionators, where each index is a member of the right foot led group
    vector<int> m_all_led_ids;                      //!< a vector of every index into LedActionators 
};

#endif

