/*! @file NAOActionators.cpp
    @brief Implementation of NAO actionators class

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

#include "NAOActionators.h"
#include "NAOActionatorNames.h"
#include "NUPlatform/NUSystem.h"

#include "debug.h"
#include "debugverbositynuactionators.h"

// init m_actionator_names:
static string temp_servo_control_names[] = {string("JointPositions")};
vector<string> NAOActionators::m_servo_control_names(temp_servo_control_names, temp_servo_control_names + sizeof(temp_servo_control_names)/sizeof(*temp_servo_control_names));

// init m_servo_names:
static string temp_servo_names[] = {HEAD_PITCH, HEAD_YAW, L_SHOULDER_ROLL, L_SHOULDER_PITCH, L_ELBOW_ROLL, L_ELBOW_YAW, R_SHOULDER_ROLL, R_SHOULDER_PITCH, R_ELBOW_ROLL, R_ELBOW_YAW, L_HIP_ROLL, L_HIP_PITCH, L_HIP_YAWPITCH, L_KNEE_PITCH, L_ANKLE_ROLL, L_ANKLE_PITCH, R_HIP_ROLL, R_HIP_PITCH, R_HIP_YAWPITCH, R_KNEE_PITCH, R_ANKLE_ROLL, R_ANKLE_PITCH};
vector<string> NAOActionators::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

// init m_servo_position_names:
static string temp_servo_position_names[] = {DN_HEAD_PITCH_POSITION, DN_HEAD_YAW_POSITION, DN_L_SHOULDER_ROLL_POSITION, DN_L_SHOULDER_PITCH_POSITION, DN_L_ELBOW_ROLL_POSITION, DN_L_ELBOW_YAW_POSITION, DN_R_SHOULDER_ROLL_POSITION, DN_R_SHOULDER_PITCH_POSITION, DN_R_ELBOW_ROLL_POSITION, DN_R_ELBOW_YAW_POSITION, DN_L_HIP_ROLL_POSITION, DN_L_HIP_PITCH_POSITION, DN_L_HIP_YAWPITCH_POSITION, DN_L_KNEE_PITCH_POSITION, DN_L_ANKLE_ROLL_POSITION, DN_L_ANKLE_PITCH_POSITION, DN_R_HIP_ROLL_POSITION, DN_R_HIP_PITCH_POSITION, DN_R_HIP_YAWPITCH_POSITION, DN_R_KNEE_PITCH_POSITION, DN_R_ANKLE_ROLL_POSITION, DN_R_ANKLE_PITCH_POSITION};
vector<string> NAOActionators::m_servo_position_names(temp_servo_position_names, temp_servo_position_names + sizeof(temp_servo_position_names)/sizeof(*temp_servo_position_names));
unsigned int NAOActionators::m_num_servo_positions = NAOActionators::m_servo_position_names.size();

// init m_servo_stiffness_names:
static string temp_servo_stiffness_names[] = {DN_HEAD_PITCH_HARDNESS, DN_HEAD_YAW_HARDNESS, DN_L_SHOULDER_ROLL_HARDNESS, DN_L_SHOULDER_PITCH_HARDNESS, DN_L_ELBOW_ROLL_HARDNESS, DN_L_ELBOW_YAW_HARDNESS, DN_R_SHOULDER_ROLL_HARDNESS, DN_R_SHOULDER_PITCH_HARDNESS, DN_R_ELBOW_ROLL_HARDNESS, DN_R_ELBOW_YAW_HARDNESS, DN_L_HIP_ROLL_HARDNESS, DN_L_HIP_PITCH_HARDNESS, DN_L_HIP_YAWPITCH_HARDNESS, DN_L_KNEE_PITCH_HARDNESS, DN_L_ANKLE_ROLL_HARDNESS, DN_L_ANKLE_PITCH_HARDNESS, DN_R_HIP_ROLL_HARDNESS, DN_R_HIP_PITCH_HARDNESS, DN_R_HIP_YAWPITCH_HARDNESS, DN_R_KNEE_PITCH_HARDNESS, DN_R_ANKLE_ROLL_HARDNESS, DN_R_ANKLE_PITCH_HARDNESS};
vector<string> NAOActionators::m_servo_stiffness_names(temp_servo_stiffness_names, temp_servo_stiffness_names + sizeof(temp_servo_stiffness_names)/sizeof(*temp_servo_stiffness_names));
unsigned int NAOActionators::m_num_servo_stiffnesses = NAOActionators::m_servo_stiffness_names.size();

// init m_earled_names:
static string temp_earled_names[] = {  DN_LED_EAR_LEFT_0DEG, DN_LED_EAR_LEFT_36DEG, DN_LED_EAR_LEFT_72DEG, DN_LED_EAR_LEFT_108DEG, DN_LED_EAR_LEFT_144DEG, DN_LED_EAR_LEFT_180DEG, DN_LED_EAR_LEFT_216DEG, DN_LED_EAR_LEFT_252DEG, DN_LED_EAR_LEFT_288DEG, DN_LED_EAR_LEFT_324DEG, \
                                       DN_LED_EAR_RIGHT_0DEG, DN_LED_EAR_RIGHT_36DEG, DN_LED_EAR_RIGHT_72DEG, DN_LED_EAR_RIGHT_108DEG, DN_LED_EAR_RIGHT_144DEG, DN_LED_EAR_RIGHT_180DEG, DN_LED_EAR_RIGHT_216DEG, DN_LED_EAR_RIGHT_252DEG, DN_LED_EAR_RIGHT_288DEG, DN_LED_EAR_RIGHT_324DEG};
vector<string> NAOActionators::m_earled_names(temp_earled_names, temp_earled_names + sizeof(temp_earled_names)/sizeof(*temp_earled_names));
unsigned int NAOActionators::m_num_earleds = NAOActionators::m_earled_names.size();

// init m_eyeled_names:
static string temp_eyeled_names[] = {  DN_LED_EYE_LEFT_RED_0DEG, DN_LED_EYE_LEFT_GREEN_0DEG, DN_LED_EYE_LEFT_BLUE_0DEG, DN_LED_EYE_LEFT_RED_45DEG, DN_LED_EYE_LEFT_GREEN_45DEG, DN_LED_EYE_LEFT_BLUE_45DEG, DN_LED_EYE_LEFT_RED_90DEG, DN_LED_EYE_LEFT_GREEN_90DEG, DN_LED_EYE_LEFT_BLUE_90DEG, DN_LED_EYE_LEFT_RED_135DEG, DN_LED_EYE_LEFT_GREEN_135DEG, DN_LED_EYE_LEFT_BLUE_135DEG, DN_LED_EYE_LEFT_RED_180DEG, DN_LED_EYE_LEFT_GREEN_180DEG, DN_LED_EYE_LEFT_BLUE_180DEG, DN_LED_EYE_LEFT_RED_225DEG, DN_LED_EYE_LEFT_GREEN_225DEG, DN_LED_EYE_LEFT_BLUE_225DEG, DN_LED_EYE_LEFT_RED_270DEG, DN_LED_EYE_LEFT_GREEN_270DEG, DN_LED_EYE_LEFT_BLUE_270DEG, DN_LED_EYE_LEFT_RED_315DEG, DN_LED_EYE_LEFT_GREEN_315DEG, DN_LED_EYE_LEFT_BLUE_315DEG, \
                                       DN_LED_EYE_RIGHT_RED_0DEG, DN_LED_EYE_RIGHT_GREEN_0DEG, DN_LED_EYE_RIGHT_BLUE_0DEG, DN_LED_EYE_RIGHT_RED_45DEG, DN_LED_EYE_RIGHT_GREEN_45DEG, DN_LED_EYE_RIGHT_BLUE_45DEG, DN_LED_EYE_RIGHT_RED_90DEG, DN_LED_EYE_RIGHT_GREEN_90DEG, DN_LED_EYE_RIGHT_BLUE_90DEG, DN_LED_EYE_RIGHT_RED_135DEG, DN_LED_EYE_RIGHT_GREEN_135DEG, DN_LED_EYE_RIGHT_BLUE_135DEG, DN_LED_EYE_RIGHT_RED_180DEG, DN_LED_EYE_RIGHT_GREEN_180DEG, DN_LED_EYE_RIGHT_BLUE_180DEG, DN_LED_EYE_RIGHT_RED_225DEG, DN_LED_EYE_RIGHT_GREEN_225DEG, DN_LED_EYE_RIGHT_BLUE_225DEG, DN_LED_EYE_RIGHT_RED_270DEG, DN_LED_EYE_RIGHT_GREEN_270DEG, DN_LED_EYE_RIGHT_BLUE_270DEG, DN_LED_EYE_RIGHT_RED_315DEG, DN_LED_EYE_RIGHT_GREEN_315DEG, DN_LED_EYE_RIGHT_BLUE_315DEG};
vector<string> NAOActionators::m_eyeled_names(temp_eyeled_names, temp_eyeled_names + sizeof(temp_eyeled_names)/sizeof(*temp_eyeled_names));
unsigned int NAOActionators::m_num_eyeleds = NAOActionators::m_eyeled_names.size();

// init m_chestled_names
static string temp_chestled_names[] = { DN_LED_CHEST_RED, DN_LED_CHEST_GREEN, DN_LED_CHEST_BLUE};
vector<string> NAOActionators::m_chestled_names(temp_chestled_names, temp_chestled_names + sizeof(temp_chestled_names)/sizeof(*temp_chestled_names));
unsigned int NAOActionators::m_num_chestleds = NAOActionators::m_chestled_names.size();

// init m_footled_names

static string temp_footled_names[] = {  DN_LED_FOOT_LEFT_RED, DN_LED_FOOT_LEFT_GREEN, DN_LED_FOOT_LEFT_BLUE, \
                                        DN_LED_FOOT_RIGHT_RED, DN_LED_FOOT_RIGHT_GREEN, DN_LED_FOOT_RIGHT_BLUE};
vector<string> NAOActionators::m_footled_names(temp_footled_names, temp_footled_names + sizeof(temp_footled_names)/sizeof(*temp_footled_names));
unsigned int NAOActionators::m_num_footleds = NAOActionators::m_footled_names.size();

vector<string> NAOActionators::m_led_names;
unsigned int NAOActionators::m_num_leds;

vector<string> NAOActionators::m_actionator_names;
unsigned int NAOActionators::m_num_actionators;

NAOActionators::NAOActionators()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOActionators::NAOActionators()" << endl;
#endif
    
    getActionatorsFromAldebaran();
    
    m_data->setAvailableJointControlMethods(m_servo_control_names);
    m_data->setAvailableJoints(m_servo_position_names);
    m_data->setAvailableLeds(m_led_names);
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 0
    debug << "NAOActionators::NAOActionators(). Avaliable Actionators: " << endl;
    m_data->summaryTo(debug);
#endif
}

NAOActionators::~NAOActionators()
{
}

void NAOActionators::getActionatorsFromAldebaran()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOActionators::getActionatorsFromALDCM()" << endl;
#endif
    m_al_dcm = new DCMProxy(NUNAO::m_broker);
    m_al_time_offset = m_al_dcm->getTime(0) - nusystem->getTime();       // so when talking to motors use time + m_al_time_offset
    ALValue param;
    param.arraySetSize(2);

    param[0] = ALIAS_POSITION;
    param[1] = m_servo_position_names;
    param = m_al_dcm->createAlias(param);
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << param.toString(VerbosityMini) << endl;
    #endif
    
    param[0] = ALIAS_STIFFNESS;
    param[1] = m_servo_stiffness_names;
    param = m_al_dcm->createAlias(param);
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << param.toString(VerbosityMini) << endl;
    #endif
    
    param[0] = ALIAS_LED;
    m_led_names.insert(m_led_names.end(), m_earled_names.begin(), m_earled_names.end());
    m_led_names.insert(m_led_names.end(), m_eyeled_names.begin(), m_eyeled_names.end());
    m_led_names.insert(m_led_names.end(), m_chestled_names.begin(), m_chestled_names.end());
    m_led_names.insert(m_led_names.end(), m_footled_names.begin(), m_footled_names.end());
    m_num_leds = m_led_names.size();
    param[1] = m_led_names;
    param = m_al_dcm->createAlias(param);
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << param.toString(VerbosityMini) << endl;
    #endif
    
    param[0] = ALIAS_ALL;
    m_actionator_names.insert(m_actionator_names.end(), m_servo_position_names.begin(), m_servo_position_names.end());
    m_actionator_names.insert(m_actionator_names.end(), m_servo_stiffness_names.begin(), m_servo_stiffness_names.end());
    m_actionator_names.insert(m_actionator_names.end(), m_led_names.begin(), m_led_names.end());
    m_num_actionators = m_actionator_names.size();
    param[1] = m_actionator_names;
    param = m_al_dcm->createAlias(param);
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << param.toString(VerbosityMini) << endl;
    #endif
    
    createALDCMCommands();
}

void NAOActionators::createALDCMCommands()
{
    createALDCMCommand(ALIAS_POSITION, m_position_command, m_num_servo_positions);
    createALDCMCommand(ALIAS_STIFFNESS, m_stiffness_command, m_num_servo_stiffnesses);
    createALDCMCommand(ALIAS_LED, m_led_command, m_num_leds);
    createALDCMCommand(ALIAS_ALL, m_actionator_command, m_num_actionators);
}

void NAOActionators::createALDCMCommand(const char* p_name, ALValue& p_command, unsigned int numactionators)
{
    // The format for a command to the DCM in alias mode is the following:
    // [AliasName, "ClearAfter", "time-mixed",[actionator0, actionator1, ... , actionatorn]]
    //      where each actionator is [[command0, dcmtime0, importance0], [command1, dcmtime1, importance1], ...]
    // However, as we only ever give the dcm one command at a time, an actionator is always [[command0, dcmtime0, importance0]]
    ALValue l_command;
    
    // time-mixed mode always has a command length of 4
    l_command.arraySetSize(4);
    l_command[0] = p_name;
    l_command[1] = "ClearAfter";
    l_command[2] = "time-mixed";
    
    ALValue actionators;
    actionators.arraySetSize(numactionators);
    ALValue points;             // the list of commands for each actionator
    points.arraySetSize(1);     // we always send only one command 
    ALValue point;              // the actionator value
    point.arraySetSize(2);      // a actionator point is always [value, dcmtime]
    for (unsigned int i=0; i<numactionators; i++)
    {
        point[0] = 0.0f;                      // value
        point[1] = m_al_dcm->getTime(0);     // time
        points[0] = point;
        actionators[i] = points;
    }
    l_command[3] = actionators;
    p_command = l_command;
}

/*! @brief Copies the commands stored in the NUActionators to the ALDcm.
 
    As of 8/4/2010 this function uses 17% of the CPU
 */
void NAOActionators::copyToHardwareCommunications()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 0
    debug << "NAOActionators::copyToHardwareCommunications()" << endl;
#endif
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    m_data->summaryTo(debug);
#endif
    
    static vector<bool> isvalid;
    static vector<double> times;
    
    static vector<float> positions;
    static vector<float> velocities;
    static vector<float> gains;
    
    if (m_data->getNextJointPositions(isvalid, times, positions, velocities, gains))
    {
        if (m_num_servo_positions == isvalid.size())                          // only process the input if it has the right length
        {
            for (unsigned int i=0; i<m_num_servo_positions; i++)
            {
                if (isvalid[i] == true)
                {
                    int time = static_cast<int> (times[i] + m_al_time_offset);
                    m_actionator_command[3][i][0][0] = positions[i];
                    m_actionator_command[3][i+m_num_servo_positions][0][0] = gains[i]/100.0;
                    m_actionator_command[3][i][0][1] = time; 
                    m_actionator_command[3][i+m_num_servo_positions][0][1] = time;
                }
                else
                {
                    m_actionator_command[3][i][0][0] = NAN;
                    m_actionator_command[3][i+m_num_servo_positions][0][0] = NAN;
                }
            }
        }
        else
            errorlog << "NAOActionators::copyToHardwareCommunications(). The positions do not have the correct length, all data will be ignored!" << endl;
    }
    
    static vector<float> redleds, greenleds, blueleds;
    
    if (m_data->getNextLeds(isvalid, times, redleds, greenleds, blueleds))
    {
        unsigned int dcmoffset = 0;
        unsigned int actoffset = 0;
        unsigned int ledoffset = 2*m_num_servo_positions;
        
        // On the NAO the ears only have blue leds
        for (unsigned int i=0; i<m_num_earleds; i++)
        {
            int j = i + ledoffset;
            int time = static_cast<int> (times[i] + m_al_time_offset);
            m_actionator_command[3][j][0][0] = blueleds[i];
            m_actionator_command[3][j][0][1] = time;
        }
        dcmoffset = actoffset = m_num_earleds;
        
        // On the NAO the eyes are red, green and blue leds
        // The left eye the blue is stuck on, on the right eye the blue and green are stuck on    
        unsigned int num = m_num_eyeleds/3;
        for (unsigned int i=0; i<num; i++)
        {
            int j = 3*i + dcmoffset + ledoffset;
            int k = i + actoffset;
            int time = static_cast<int> (times[k] + m_al_time_offset);
            m_actionator_command[3][j][0][0] = redleds[k];
            m_actionator_command[3][j+1][0][0] = greenleds[k];
            m_actionator_command[3][j+2][0][0] = blueleds[k];
            m_actionator_command[3][j][0][1] = time;
            m_actionator_command[3][j+1][0][1] = time;
            m_actionator_command[3][j+2][0][1] = time;
        }
        dcmoffset += m_num_eyeleds;
        actoffset += m_num_eyeleds/3;
        
        // On the NAO the chest has a red, green and blue led
        num = m_num_chestleds/3;
        for (unsigned int i=0; i<num; i++)
        {
            int j = 3*i + dcmoffset + ledoffset;
            int k = i + actoffset;
            int time = static_cast<int> (times[k] + m_al_time_offset);
            m_actionator_command[3][j][0][0] = redleds[k];
            m_actionator_command[3][j+1][0][0] = greenleds[k];
            m_actionator_command[3][j+2][0][0] = blueleds[k];
            m_actionator_command[3][j][0][1] = time;
            m_actionator_command[3][j+1][0][1] = time;
            m_actionator_command[3][j+2][0][1] = time;
        }
        dcmoffset += m_num_chestleds;
        actoffset += m_num_chestleds/3;
        
        // On the NAO the feet each have red, green and blue leds
        num = m_num_footleds/3;
        for (unsigned int i=0; i<num; i++)
        {
            int j = 3*i + dcmoffset + ledoffset;
            int k = i + actoffset;
            int time = static_cast<int> (times[k] + m_al_time_offset);
            m_actionator_command[3][j][0][0] = redleds[k];
            m_actionator_command[3][j+1][0][0] = greenleds[k];
            m_actionator_command[3][j+2][0][0] = blueleds[k];
            m_actionator_command[3][j][0][1] = time;
            m_actionator_command[3][j+1][0][1] = time;
            m_actionator_command[3][j+2][0][1] = time;
        }
    }
    
    // Setting the alias for stiffness, position and leds separately takes 1.05ms
    m_al_dcm->setAlias(m_actionator_command);
    
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << m_actionator_command.toString(VerbosityMini) << endl;
    #endif
    
    m_data->removeCompletedPoints(m_current_time);
}




