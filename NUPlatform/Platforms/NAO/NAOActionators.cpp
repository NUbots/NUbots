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
#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

#include "debug.h"
#include "debugverbositynuactionators.h"
#include "Tools/Profiling/Profiler.h"

#include <limits>
using namespace std;

// init m_servo_position_names:
static string temp_servo_position_names[] = {HEAD_PITCH_POSITION, HEAD_YAW_POSITION, L_SHOULDER_ROLL_POSITION, L_SHOULDER_PITCH_POSITION, L_ELBOW_ROLL_POSITION, L_ELBOW_YAW_POSITION, R_SHOULDER_ROLL_POSITION, R_SHOULDER_PITCH_POSITION, R_ELBOW_ROLL_POSITION, R_ELBOW_YAW_POSITION, L_HIP_ROLL_POSITION, L_HIP_PITCH_POSITION, L_HIP_YAWPITCH_POSITION, L_KNEE_PITCH_POSITION, L_ANKLE_ROLL_POSITION, L_ANKLE_PITCH_POSITION, R_HIP_ROLL_POSITION, R_HIP_PITCH_POSITION, R_HIP_YAWPITCH_POSITION, R_KNEE_PITCH_POSITION, R_ANKLE_ROLL_POSITION, R_ANKLE_PITCH_POSITION};
vector<string> NAOActionators::m_servo_position_names(temp_servo_position_names, temp_servo_position_names + sizeof(temp_servo_position_names)/sizeof(*temp_servo_position_names));
unsigned int NAOActionators::m_num_servo_positions = NAOActionators::m_servo_position_names.size();

// init m_servo_stiffness_names:
static string temp_servo_stiffness_names[] = {HEAD_PITCH_HARDNESS, HEAD_YAW_HARDNESS, L_SHOULDER_ROLL_HARDNESS, L_SHOULDER_PITCH_HARDNESS, L_ELBOW_ROLL_HARDNESS, L_ELBOW_YAW_HARDNESS, R_SHOULDER_ROLL_HARDNESS, R_SHOULDER_PITCH_HARDNESS, R_ELBOW_ROLL_HARDNESS, R_ELBOW_YAW_HARDNESS, L_HIP_ROLL_HARDNESS, L_HIP_PITCH_HARDNESS, L_HIP_YAWPITCH_HARDNESS, L_KNEE_PITCH_HARDNESS, L_ANKLE_ROLL_HARDNESS, L_ANKLE_PITCH_HARDNESS, R_HIP_ROLL_HARDNESS, R_HIP_PITCH_HARDNESS, R_HIP_YAWPITCH_HARDNESS, R_KNEE_PITCH_HARDNESS, R_ANKLE_ROLL_HARDNESS, R_ANKLE_PITCH_HARDNESS};
vector<string> NAOActionators::m_servo_stiffness_names(temp_servo_stiffness_names, temp_servo_stiffness_names + sizeof(temp_servo_stiffness_names)/sizeof(*temp_servo_stiffness_names));
unsigned int NAOActionators::m_num_servo_stiffnesses = NAOActionators::m_servo_stiffness_names.size();

// init m_earled_names:
static string temp_earled_names[] = {  LED_EAR_LEFT_0DEG, LED_EAR_LEFT_36DEG, LED_EAR_LEFT_72DEG, LED_EAR_LEFT_108DEG, LED_EAR_LEFT_144DEG, LED_EAR_LEFT_180DEG, LED_EAR_LEFT_216DEG, LED_EAR_LEFT_252DEG, LED_EAR_LEFT_288DEG, LED_EAR_LEFT_324DEG, \
                                       LED_EAR_RIGHT_0DEG, LED_EAR_RIGHT_36DEG, LED_EAR_RIGHT_72DEG, LED_EAR_RIGHT_108DEG, LED_EAR_RIGHT_144DEG, LED_EAR_RIGHT_180DEG, LED_EAR_RIGHT_216DEG, LED_EAR_RIGHT_252DEG, LED_EAR_RIGHT_288DEG, LED_EAR_RIGHT_324DEG};
vector<string> NAOActionators::m_earled_names(temp_earled_names, temp_earled_names + sizeof(temp_earled_names)/sizeof(*temp_earled_names));
unsigned int NAOActionators::m_num_earleds = NAOActionators::m_earled_names.size();

// init m_eyeled_names:
static string temp_eyeled_names[] = {  LED_EYE_LEFT_RED_0DEG, LED_EYE_LEFT_GREEN_0DEG, LED_EYE_LEFT_BLUE_0DEG, LED_EYE_LEFT_RED_45DEG, LED_EYE_LEFT_GREEN_45DEG, LED_EYE_LEFT_BLUE_45DEG, LED_EYE_LEFT_RED_90DEG, LED_EYE_LEFT_GREEN_90DEG, LED_EYE_LEFT_BLUE_90DEG, LED_EYE_LEFT_RED_135DEG, LED_EYE_LEFT_GREEN_135DEG, LED_EYE_LEFT_BLUE_135DEG, LED_EYE_LEFT_RED_180DEG, LED_EYE_LEFT_GREEN_180DEG, LED_EYE_LEFT_BLUE_180DEG, LED_EYE_LEFT_RED_225DEG, LED_EYE_LEFT_GREEN_225DEG, LED_EYE_LEFT_BLUE_225DEG, LED_EYE_LEFT_RED_270DEG, LED_EYE_LEFT_GREEN_270DEG, LED_EYE_LEFT_BLUE_270DEG, LED_EYE_LEFT_RED_315DEG, LED_EYE_LEFT_GREEN_315DEG, LED_EYE_LEFT_BLUE_315DEG, \
                                       LED_EYE_RIGHT_RED_0DEG, LED_EYE_RIGHT_GREEN_0DEG, LED_EYE_RIGHT_BLUE_0DEG, LED_EYE_RIGHT_RED_45DEG, LED_EYE_RIGHT_GREEN_45DEG, LED_EYE_RIGHT_BLUE_45DEG, LED_EYE_RIGHT_RED_90DEG, LED_EYE_RIGHT_GREEN_90DEG, LED_EYE_RIGHT_BLUE_90DEG, LED_EYE_RIGHT_RED_135DEG, LED_EYE_RIGHT_GREEN_135DEG, LED_EYE_RIGHT_BLUE_135DEG, LED_EYE_RIGHT_RED_180DEG, LED_EYE_RIGHT_GREEN_180DEG, LED_EYE_RIGHT_BLUE_180DEG, LED_EYE_RIGHT_RED_225DEG, LED_EYE_RIGHT_GREEN_225DEG, LED_EYE_RIGHT_BLUE_225DEG, LED_EYE_RIGHT_RED_270DEG, LED_EYE_RIGHT_GREEN_270DEG, LED_EYE_RIGHT_BLUE_270DEG, LED_EYE_RIGHT_RED_315DEG, LED_EYE_RIGHT_GREEN_315DEG, LED_EYE_RIGHT_BLUE_315DEG};
vector<string> NAOActionators::m_eyeled_names(temp_eyeled_names, temp_eyeled_names + sizeof(temp_eyeled_names)/sizeof(*temp_eyeled_names));
unsigned int NAOActionators::m_num_eyeleds = NAOActionators::m_eyeled_names.size();

// init m_chestled_names
static string temp_chestled_names[] = { LED_CHEST_RED, LED_CHEST_GREEN, LED_CHEST_BLUE};
vector<string> NAOActionators::m_chestled_names(temp_chestled_names, temp_chestled_names + sizeof(temp_chestled_names)/sizeof(*temp_chestled_names));
unsigned int NAOActionators::m_num_chestleds = NAOActionators::m_chestled_names.size();

// init m_footled_names
static string temp_footled_names[] = {  LED_FOOT_LEFT_RED, LED_FOOT_LEFT_GREEN, LED_FOOT_LEFT_BLUE, \
                                        LED_FOOT_RIGHT_RED, LED_FOOT_RIGHT_GREEN, LED_FOOT_RIGHT_BLUE};
vector<string> NAOActionators::m_footled_names(temp_footled_names, temp_footled_names + sizeof(temp_footled_names)/sizeof(*temp_footled_names));
unsigned int NAOActionators::m_num_footleds = NAOActionators::m_footled_names.size();

vector<string> NAOActionators::m_led_names;
unsigned int NAOActionators::m_num_leds;

// init m_other_names
static string temp_other_names[] = {"Sound"};
vector<string> NAOActionators::m_other_names(temp_other_names, temp_other_names + sizeof(temp_other_names)/sizeof(*temp_other_names));
unsigned int NAOActionators::m_num_others = NAOActionators::m_other_names.size();

vector<string> NAOActionators::m_actionator_names;
unsigned int NAOActionators::m_num_actionators;

NAOActionators::NAOActionators()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOActionators::NAOActionators()" << endl;
#endif
    
    getActionatorsFromAldebaran();
    
	vector<string> names;
    names.insert(names.end(), m_servo_position_names.begin(), m_servo_position_names.end());
    names.insert(names.end(), m_earled_names.begin(), m_earled_names.end());
    names.insert(names.end(), m_eyeled_names.begin(), m_eyeled_names.end());
    names.insert(names.end(), m_chestled_names.begin(), m_chestled_names.end());
    names.insert(names.end(), m_footled_names.begin(), m_footled_names.end());
    names.insert(names.end(), m_other_names.begin(), m_other_names.end());
    m_data->addActionators(names);
    
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
    m_al_time_offset = m_al_dcm->getTime(0) - Platform->getTime();       // so when talking to motors use time + m_al_time_offset
    
    // Create actionator aliases
    ALValue param;
    param.arraySetSize(2);      // an alias always has length two [aliasname, [device0, device1, device2, ..., deviceN]

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
    
    param[0] = ALIAS_POSITION_AND_STIFFNESS;
    vector<string> positionstiffness_names;
	positionstiffness_names.insert(positionstiffness_names.end(), m_servo_position_names.begin(), m_servo_position_names.end());
    positionstiffness_names.insert(positionstiffness_names.end(), m_servo_stiffness_names.begin(), m_servo_stiffness_names.end());
    param[1] = positionstiffness_names;
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
    startUltrasonics();
}

/*! @brief Starts the ultrasonic sensors in periodic left and right mode
 */
void NAOActionators::startUltrasonics()
{
    // nowadays the ultrasonics need only be started in periodic mode:
    // the distances are put in US/Sensor/Right/Value and US/Sensor/Left/Value
    ALValue command;            // [actuatorname, "ClearAll", [[value, time]] ]
    command.arraySetSize(3);
    command[0] = "US/Actuator/Value";
    command[1] = "ClearAll";
    
    ALValue params, param;
    params.arraySetSize(1);
    param.arraySetSize(2);
    param[0] = 68.0;                    // left/right periodic mode = 64.0 + 4.0
    param[1] = m_al_dcm->getTime(0);
    params[0] = param;
    command[2] = params;
    
    m_al_dcm->set(command);
}

void NAOActionators::createALDCMCommands()
{
    createALDCMCommand(ALIAS_POSITION, m_position_command, m_num_servo_positions);
    createALDCMCommand(ALIAS_STIFFNESS, m_stiffness_command, m_num_servo_stiffnesses);
    createALDCMCommand(ALIAS_LED, m_led_command, m_num_leds);   
    createALDCMCommand(ALIAS_POSITION_AND_STIFFNESS, m_positionstiffness_command, m_num_servo_positions + m_num_servo_stiffnesses);
    createALDCMCommand(ALIAS_ALL, m_actionator_command, m_num_actionators);
}

void NAOActionators::createALDCMCommand(const char* p_name, ALValue& p_command, unsigned int numactionators)
{
    // The format for a command to the DCM in alias mode is the following:
    // [AliasName, "ClearAfter", "time-separate", 1 (importance), [[time]], [actionator0, actionator1, ... , actionatorn]]    
    //		where each actionator is [command0, command1, ..]. However, we send only a single command at a time, so its just [command]
    ALValue l_command;
    
    // time-separate mode always has a command length of 6
    l_command.arraySetSize(6);
    l_command[0] = string(p_name);					// AliasName
    l_command[1] = string("ClearAfter");			// Update mode
    l_command[2] = string("time-separate");			// Command format	
    l_command[3] = 0;								// Importance level
    l_command[4].arraySetSize(1);
    l_command[4][0] = m_al_dcm->getTime(0);			// Time
    l_command[5].arraySetSize(numactionators);		// Actual actionator data
    
    for (unsigned int i=0; i<numactionators; i++)
    {
        l_command[5][i].arraySetSize(1);
        l_command[5][i][0] = 0.0f;					// The single actionator value
    }
    
    p_command = l_command;
}

/*! @brief Copies the commands stored in the NUActionators to the ALDcm.
 
    As of 8/4/2010 this function takes 1.7ms to complete
 	As of 28/11/2010 this function takes about 1.55ms to complete (time-mixed)
 	As of 29/11/2010 this function takes about 1.0ms to complete (time-separate) 
 	As of 29/11/2020 this function takes about 0.50ms to complete (time-separate, leds at 0.25 dcm rate)
 */
void NAOActionators::copyToHardwareCommunications()
{
    #if DEBUG_NUACTIONATORS_VERBOSITY > 0
        debug << "NAOActionators::copyToHardwareCommunications()" << endl;
    #endif
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        m_data->summaryTo(debug);
    #endif
    
    static vector<float> positions;
    static vector<float> gains;
    
    m_data->getNextServos(positions, gains);
    
    int time = m_al_dcm->getTime(0);
    m_position_command[4][0] = time;
    m_stiffness_command[4][0] = time;
    
    size_t n = positions.size();
    for (size_t i=0; i<n; i++)
    {
        m_position_command[5][i][0] = positions[i];
        m_stiffness_command[5][i][0] = gains[i]/100.0;
    }
    m_al_dcm->setAlias(m_position_command);
    m_al_dcm->setAlias(m_stiffness_command);
    
    static vector<vector<float> > ledvalues;
    static unsigned int count = 0;
    if (count%5 == 0)
    {
        m_data->getNextLeds(ledvalues);
     	
        m_led_command[4][0] = time;
        
        /*for (size_t i=0; i<ledvalues.size(); i++)
        {
            unsigned int dcmoffset = 0;
            unsigned int actoffset = 0;
            unsigned int ledoffset = 2*m_num_servo_positions;
            
            // On the NAO the ears only have blue leds
            for (unsigned int i=0; i<m_num_earleds; i++)
            {
                int j = i + ledoffset;
                m_actionator_command[3][j][0][0] = ledvalues[i][2];
                m_actionator_command[3][j][0][1] = time;
            }
            dcmoffset = actoffset = m_num_earleds;
            
            // On the NAO the eyes are red, green and blue leds 
            unsigned int num = m_num_eyeleds/3;
            for (unsigned int i=0; i<num; i++)
            {
                int j = 3*i + dcmoffset + ledoffset;
                int k = i + actoffset;
                m_actionator_command[3][j][0][0] = ledvalues[k][0];
                m_actionator_command[3][j+1][0][0] = ledvalues[k][1];
                m_actionator_command[3][j+2][0][0] = ledvalues[k][2];
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
                m_actionator_command[3][j][0][0] = ledvalues[k][0];
                m_actionator_command[3][j+1][0][0] = ledvalues[k][1];
                m_actionator_command[3][j+2][0][0] = ledvalues[k][2];
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
                m_actionator_command[3][j][0][0] = ledvalues[k][0];
                m_actionator_command[3][j+1][0][0] = ledvalues[k][1];
                m_actionator_command[3][j+2][0][0] = ledvalues[k][2];
                m_actionator_command[3][j][0][1] = time;
                m_actionator_command[3][j+1][0][1] = time;
                m_actionator_command[3][j+2][0][1] = time;
            }
        }*/
        m_al_dcm->setAlias(m_led_command);
    }
    count++;
    
    #if DEBUG_NUACTIONATORS_VERBOSITY > 4
        debug << m_position_command.toString(VerbosityMini) << endl;
    	debug << m_stiffness_command.toString(VerbosityMini) << endl;
        debug << m_led_command.toString(VerbosityMini) << endl;
    #endif

    copyToSound();
}




