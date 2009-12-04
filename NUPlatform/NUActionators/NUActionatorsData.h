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
    typedef int camera_setting_id_t;
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
    static joint_id_t TorsoRoll;
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
    // camera settings
    static camera_setting_id_t Resolution;
    static camera_setting_id_t FramesPerSecond;
    static camera_setting_id_t AutoExposure;
    static camera_setting_id_t AutoWhiteBalance;
    static camera_setting_id_t AutoGain;
    static camera_setting_id_t Brightness;
    static camera_setting_id_t Saturation;
    static camera_setting_id_t RedChroma;
    static camera_setting_id_t BlueChroma;
    static camera_setting_id_t Gain;
    static camera_setting_id_t Exposure;
    static camera_setting_id_t SelectCamera;
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
        LArm,
        RArm,
        Torso,
        LLeg,
        RLeg,
        Body,
        All
    };
public:
    NUActionatorsData();
    ~NUActionatorsData();
    
    // Methods for setting which actionators are available on init
    void setAvailableJointControlMethods(const vector<string>& methodnames);
    void setAvailableJoints(const vector<string>& jointnamess);
    void setAvailableLeds(const vector<string>& lednames);
    void setAvailableCameraSettings(const vector<string>& camerasettingnames);
    void setAvailableOtherActionators(const vector<string>& actionatornames);
    
    void removeCompletedPoints(double currenttime);
    
    // Get methods for joints
    bool getNextJointPositions(vector<bool>& isvalid, vector<double>& time, vector<float>& positions, vector<float>& velocities, vector<float>& gains);
    bool getNextJointTorques(vector<bool>& isvalid, vector<double>& time, vector<float>& torques, vector<float>& gains);
    
    bool getNextCameraSettings(vector<bool>& isvalid, vector<double>& time, vector<vector<float> >& data);
    bool getNextLeds(vector<bool>& isvalid, vector<double>& time, vector<float>& redvalues, vector<float>& greenvalues, vector<float>& bluevalues);
    bool getNextSound(bool& isvalid, double& time, int& soundid, string& text);
    
    // Methods for adding new position or torque values for a single joint
    bool addJointPosition(joint_id_t jointid, double time, float position, float velocity, float gain);
    bool addJointTorque(joint_id_t jointid, double time, float torque, float gain);
    bool addLed(led_id_t ledid, double time, float redvalue, float greenvalue, float bluevalue); 
    bool addCameraSetting(camera_setting_id_t settingid, double time, vector<float>& data);
    
    // Methods for adding new position or torque values for a body part
    bool addJointPositions(bodypart_id_t partid, double time, const vector<float>& positions, const vector<float>& velocities, const vector<float>& gains);
    bool addJointTorques(bodypart_id_t partid, double time, const vector<float>& torques, const vector<float>& gains);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUActionatorsData& p_sensor);
    friend istream& operator>> (istream& input, NUActionatorsData& p_sensor);
    
private:
    void addJointActionator(string actionatorname);
    void addCameraSettingActionator(string actionatorname);
    void addLedActionator(string actionatorname);
    void addActionator(vector<actionator_t*>& actionatorgroup, string actionatorname, actionator_t::actionator_type_t actionatortype); 
    void addActionator(actionator_t*& p_actionator, string actionatorname, actionator_t::actionator_type_t actionatortype);
    string simplifyName(const string& input);
    void simplifyNames(const vector<string>& input, vector<string>& output);
    
private:
    // Limb position and torque actionators
    bool m_positionactionation;
    bool m_torqueactionation;
    vector<actionator_t*> PositionActionators;      //!< the actionators to change the position, velocity and postion-gain
    vector<actionator_t*> TorqueActionators;        //!< the actionators to change the torque, and torque-gain

    // Peripheral actionators 
    vector<actionator_t*> CameraActionators;        //!< The camera control actionators, a single actionator for each setting Steve.
    vector<actionator_t*> LedActionators;           //!< The led actionators
    actionator_t* Sound;                      //!< The sound actionator
    
    vector<actionator_t*> m_all_actionators;
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

