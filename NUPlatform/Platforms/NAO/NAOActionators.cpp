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

// init m_led_names:
static string temp_led_names[] = {};
vector<string> NAOActionators::m_led_names(temp_led_names, temp_led_names + sizeof(temp_led_names)/sizeof(*temp_led_names));
unsigned int NAOActionators::m_num_leds = NAOActionators::m_led_names.size();

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
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 3
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
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << param.toString(VerbosityMini) << endl;
    #endif
    
    param[0] = ALIAS_STIFFNESS;
    param[1] = m_servo_stiffness_names;
    param = m_al_dcm->createAlias(param);
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << param.toString(VerbosityMini) << endl;
    #endif
    
    param[0] = ALIAS_LED;
    param[1] = m_led_names;
    param = m_al_dcm->createAlias(param);
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << param.toString(VerbosityMini) << endl;
    #endif
    
    createALDCMCommands();
}

void NAOActionators::createALDCMCommands()
{
    createALDCMCommand(ALIAS_POSITION, m_position_command, m_num_servo_positions);
    createALDCMCommand(ALIAS_STIFFNESS, m_stiffness_command, m_num_servo_stiffnesses);
    createALDCMCommand(ALIAS_LED, m_led_command, m_num_leds);
}

void NAOActionators::createALDCMCommand(const char* p_name, ALValue& p_command, unsigned int numactionators)
{
    // The format for a command to the DCM in alias mode is the following:
    // [AliasName, "ClearAfter", "time-mixed",[actionator0, actionator1, ... , actionatorn]]
    //      where each actionator is [[command0, dcmtime0, importance0], [command1, dcmtime1, importance1], ...]
    // However, as we only ever give the dcm one command at a time an actionator is always [[command0, dcmtime0, importance0]]
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
    point.arraySetSize(3);      // a actionator point is always [value, dcmtime, importance]
    for (unsigned int i=0; i<numactionators; i++)
    {
        point[0] = 0;
        point[1] = (int) 0;
        point[2] = 0;
        points[0] = point;
        actionators[i] = points;
    }
    l_command[3] = actionators;
    p_command = l_command;
}

void NAOActionators::copyToHardwareCommunications()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
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
    
    static vector<int> dcmtimes(m_num_servo_positions, m_al_dcm->getTime(0));
    static vector<float> dcmleds(m_num_leds, NAN);
    static vector<float> dcmpositions(m_num_servo_positions, NAN);
    static vector<float> dcmstiffnesses(m_num_servo_stiffnesses, NAN);
    
    if (m_data->getNextJointPositions(isvalid, times, positions, velocities, gains))
    {
        if (m_num_servo_positions == isvalid.size())                          // only process the input if it has the right length
        {
            for (unsigned int i=0; i<m_num_servo_positions; i++)
            {
                if (isvalid[i] == true)
                {
                    dcmtimes[i] = static_cast<int> (times[i] + m_al_time_offset);
                    dcmstiffnesses[i] = gains[i]/100.0;
                    dcmpositions[i] = positions[i];
                }
                else
                {
                    dcmtimes[i] = 0;
                    dcmstiffnesses[i] = NAN;
                    dcmpositions[i] = NAN;
                }
            }
        }
        else
            errorlog << "NAOActionators::copyToHardwareCommunications(). The input does not have the correct length, all data will be ignored!" << endl;
    }
    
    for (unsigned int i=0; i<m_num_servo_positions; i++)
    {
        m_stiffness_command[3][i][0][0] = dcmstiffnesses[i];
        m_stiffness_command[3][i][0][1] = dcmtimes[i];
        m_position_command[3][i][0][0] = dcmpositions[i];
        m_position_command[3][i][0][1] = dcmtimes[i];
    }
    m_al_dcm->setAlias(m_stiffness_command);
    m_al_dcm->setAlias(m_position_command);
    m_al_dcm->setAlias(m_led_command);
    
    m_data->removeCompletedPoints(m_current_time);
}




