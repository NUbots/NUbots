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
#include "debug.h"

// init m_actionator_names:
static string temp_servo_control_names[] = {string("JointPositions")};
vector<string> NAOActionators::m_servo_control_names(temp_servo_control_names, temp_servo_control_names + sizeof(temp_servo_control_names)/sizeof(*temp_servo_control_names));

// init m_servo_position_names:
static string temp_servo_position_names[] = {DN_HEAD_PITCH_POSITION, DN_HEAD_YAW_POSITION, DN_L_SHOULDER_ROLL_POSITION, DN_L_SHOULDER_PITCH_POSITION, DN_L_ELBOW_ROLL_POSITION, DN_L_ELBOW_YAW_POSITION, DN_R_SHOULDER_ROLL_POSITION, DN_R_SHOULDER_PITCH_POSITION, DN_R_ELBOW_ROLL_POSITION, DN_R_ELBOW_YAW_POSITION, DN_L_HIP_ROLL_POSITION, DN_L_HIP_PITCH_POSITION, DN_L_HIP_YAWPITCH_POSITION, DN_L_KNEE_PITCH_POSITION, DN_L_ANKLE_ROLL_POSITION, DN_L_ANKLE_PITCH_POSITION, DN_R_HIP_ROLL_POSITION, DN_R_HIP_PITCH_POSITION, DN_R_HIP_YAWPITCH_POSITION, DN_R_KNEE_PITCH_POSITION, DN_R_ANKLE_ROLL_POSITION, DN_R_ANKLE_PITCH_POSITION};
vector<string> NAOActionators::m_servo_position_names(temp_servo_position_names, temp_servo_position_names + sizeof(temp_servo_position_names)/sizeof(*temp_servo_position_names));

// init m_servo_position_names:
static string temp_servo_stiffness_names[] = {DN_HEAD_PITCH_HARDNESS, DN_HEAD_YAW_HARDNESS, DN_L_SHOULDER_ROLL_HARDNESS, DN_L_SHOULDER_PITCH_HARDNESS, DN_L_ELBOW_ROLL_HARDNESS, DN_L_ELBOW_YAW_HARDNESS, DN_R_SHOULDER_ROLL_HARDNESS, DN_R_SHOULDER_PITCH_HARDNESS, DN_R_ELBOW_ROLL_HARDNESS, DN_R_ELBOW_YAW_HARDNESS, DN_L_HIP_ROLL_HARDNESS, DN_L_HIP_PITCH_HARDNESS, DN_L_HIP_YAWPITCH_HARDNESS, DN_L_KNEE_PITCH_HARDNESS, DN_L_ANKLE_ROLL_HARDNESS, DN_L_ANKLE_PITCH_HARDNESS, DN_R_HIP_ROLL_HARDNESS, DN_R_HIP_PITCH_HARDNESS, DN_R_HIP_YAWPITCH_HARDNESS, DN_R_KNEE_PITCH_HARDNESS, DN_R_ANKLE_ROLL_HARDNESS, DN_R_ANKLE_PITCH_HARDNESS};
vector<string> NAOActionators::m_servo_stiffness_names(temp_servo_stiffness_names, temp_servo_stiffness_names + sizeof(temp_servo_stiffness_names)/sizeof(*temp_servo_stiffness_names));

// init m_led_names:
static string temp_led_names[] = {};
vector<string> NAOActionators::m_led_names(temp_led_names, temp_led_names + sizeof(temp_led_names)/sizeof(*temp_led_names));

vector<string> NAOActionators::m_actionator_names;

NAOActionators::NAOActionators()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOActionators::NAOActionators()" << endl;
#endif
    getActionatorsFromALDCM();
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

void NAOActionators::getActionatorsFromALDCM()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOActionators::getActionatorsFromALDCM()" << endl;
#endif
    m_al_dcm = new DCMProxy(NUNAO::m_broker);
    m_al_time_offset = nusystem->getTime() - m_al_dcm->getTime(0);       // so when talking to motors use time + m_al_time_offset
    ALValue param;
    
    param.arraySetSize(2);
    param[0] = ALIAS_POSITION;
    param[1] = m_servo_position_names;
    param = m_al_dcm->createAlias(param);
    debug << param.toString(VerbosityMini) << endl;
    
    param[0] = ALIAS_STIFFNESS;
    param[1] = m_servo_stiffness_names;
    param = m_al_dcm->createAlias(param);
    debug << param.toString(VerbosityMini) << endl;
    
    param[0] = ALIAS_LED;
    param[1] = m_led_names;
    param = m_al_dcm->createAlias(param);
    debug << param.toString(VerbosityMini) << endl;
}

void NAOActionators::copyToHardwareCommunications()
{
    static vector<bool> isvalid;            
    static vector<double> times;
    static vector<float> positions;
    static vector<float> velocities;
    static vector<float> gains;
    
    static vector<float> dcmtimes(m_servo_position_names.size(), m_al_dcm->getTime(0));
    static vector<float> dcmpositions(m_servo_position_names.size(), 0);
    static vector<float> dcmstiffnesses(m_servo_stiffness_names.size(), 0);
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOActionators::copyToHardwareCommunications()" << endl;
#endif
    if (m_data->getNextJointPositions(isvalid, times, positions, velocities, gains))
    {
        if (m_servo_position_names.size() == isvalid.size())                          // only process the input if it has the right length
        {
            for (unsigned int i=0; i<m_servo_position_names.size(); i++)
            {
                if (isvalid[i] == true)
                {
                    dcmtimes[i] = times[i] + m_al_time_offset;
                    dcmstiffnesses[i] = gains[i]/100.0;
                    dcmpositions[i] = positions[i];
                }
            }
        }
        else
            debug << "NAOWebotsActionators::copyToServos(). The input does not have the correct length, all data will be ignored!" << endl;
    }
    
    // This time round I am going to go with time-mixed. This uses a HUGE amount of CPU: consider revising!
    ALValue command;
    command.arraySetSize(4);
    command[0] = ALIAS_STIFFNESS;
    command[1] = "ClearAfter";
    command[2] = "time-mixed";
    
    ALValue actionators;
    actionators.arraySetSize(m_servo_stiffness_names.size());
    
    ALValue points;
    points.arraySetSize(1);
    ALValue point;
    point.arraySetSize(3);
    for (unsigned int i=0; i<m_servo_stiffness_names.size(); i++)
    {
        point[0] = dcmstiffnesses[i];
        point[1] = dcmtimes[i];
        point[2] = 0;
        points[0] = point;
        actionators[i] = points;
    }
    command[3] = actionators;
    debug << m_al_time_offset << " " << nusystem->getTime() << " " << m_al_dcm->getTime(0) << endl;
    debug << command.toString(VerbosityMini) << endl;
    m_al_dcm->setAlias(command);
}




