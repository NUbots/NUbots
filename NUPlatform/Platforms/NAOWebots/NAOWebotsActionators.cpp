/*! @file NAOWebotsActionators.cpp
    @brief Implementation of NAO in Webots actionators class

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

#include "NAOWebotsActionators.h"
#include "Tools/debug.h"

// init m_actionator_names:
static string temp_servo_control_names[] = {string("JointPositions"), string("JointTorques")};
vector<string> NAOWebotsActionators::m_servo_control_names(temp_servo_control_names, temp_servo_control_names + sizeof(temp_servo_control_names)/sizeof(*temp_servo_control_names));

// init m_servo_names:
static string temp_servo_names[] = {string("HeadYaw"), string("HeadPitch"), \
                                    string("LShoulderPitch"), string("LShoulderRoll"), string("LElbowYaw"), string("LElbowRoll"), \
                                    string("RShoulderPitch"), string("RShoulderRoll"), string("RElbowYaw"), string("RElbowRoll"), \
                                    string("LHipYawPitch"), string("LHipPitch"), string("LHipRoll"), string("LKneePitch"), string("LAnklePitch"), string("LAnkleRoll"), \
                                    string("RHipYawPitch"), string("RHipPitch"), string("RHipRoll"), string("RKneePitch"), string("RAnklePitch"), string("RAnkleRoll")};
vector<string> NAOWebotsActionators::m_servo_names(temp_servo_names, temp_servo_names + sizeof(temp_servo_names)/sizeof(*temp_servo_names));

// init m_led_names:
static string temp_led_names[] = {string("Ears/Led/Left"), string("Ears/Led/Right"), string("Face/Led/Left"), string("Face/Led/Right"), \
                                  string("ChestBoard/Led"), \
                                  string("LFoot/Led"), string("RFoot/Led")};
vector<string> NAOWebotsActionators::m_led_names(temp_led_names, temp_led_names + sizeof(temp_led_names)/sizeof(*temp_led_names));

/*! @brief Constructs a nubot actionator class with a Webots backend
 
    @param platform a pointer to the nuplatform (this is required because webots needs to have nuplatform inherit from the Robot class)
 */ 
NAOWebotsActionators::NAOWebotsActionators(NAOWebotsPlatform* platform) : m_simulation_step(platform->getBasicTimeStep())
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::NAOWebotsActionators()" << endl;
#endif
    m_platform = platform;
    getActionatorsFromWebots(platform);
    enableActionatorsInWebots();
    
    m_data->setAvailableJointControlMethods(m_servo_control_names);
    m_data->setAvailableJoints(m_servo_names);
    m_data->setAvailableLeds(m_led_names);
    //m_data->setAvailableCameraSettings();        // I am not sure if this should be *here* at all
    //m_data->setAvailableOtherActionators();      there are no other actionators at the moment 
    
    
    m_data->addJointPosition(NUActionatorsData::HeadYaw, platform->system->getTime() + 350, 0, 1, 100);
    m_data->addJointPosition(NUActionatorsData::HeadYaw, platform->system->getTime() + 4000, -1.57, 1, 100);
    m_data->addJointPosition(NUActionatorsData::HeadYaw, platform->system->getTime() + 8000, 1.57, 1, 100);
    
    vector<float> pos (2, 0);
    vector<float> vel (2, 1);
    vector<float> gain (2, 100);
    pos[1] = -0.7;
    m_data->addJointPositions(NUActionatorsData::Head, platform->system->getTime() + 10000, pos, vel, gain);
    
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 3
    debug << "NAOWebotsActionators::NAOWebotsActionators(). Avaliable Actionators: " << endl;
    m_data->summaryTo(debug);
#endif
}

/*! @brief Get pointers to each of the actionators in the simulated NAO
 */
void NAOWebotsActionators::getActionatorsFromWebots(NAOWebotsPlatform* platform)
{
    // Get the servos
    for (int i=0; i<m_servo_names.size(); i++)
        m_servos.push_back(platform->getServo(m_servo_names[i]));
    // Get the camera
    m_camera_control = platform->getServo("CameraSelect");
    // Get the leds
    for (int i=0; i<m_led_names.size(); i++)
        m_leds.push_back(platform->getLED(m_led_names[i]));
    //! @todo TODO: get the sound from webots
}

void NAOWebotsActionators::enableActionatorsInWebots()
{
    // Nothing needs to be done here!
}

NAOWebotsActionators::~NAOWebotsActionators()
{
}

void NAOWebotsActionators::copyToHardwareCommunications()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::copyToHardwareCommunications()" << endl;
#endif
    static double currenttime;
    static vector<bool> isvalid;
    static vector<double> times;
    static vector<float> positions;
    static vector<float> velocities;
    static vector<float> torques;
    static vector<float> gains;
    
    static vector<float> redvalues;
    static vector<float> greenvalues;
    static vector<float> bluevalues;
    
    currenttime = m_platform->system->getTime();
    m_data->removeCompletedPoints(currenttime);
    
    if (m_data->getNextJointPositions(isvalid, times, positions, velocities, gains))
    {
        for (int i=0; i<m_servos.size(); i++)
        {
            if (isvalid[i] == true)
            {
                if ((times[i] - currenttime) > m_simulation_step)
                {
                    float c = m_servos[i]->getPosition();          // i think I am allowed to do this right? I ought to be I am only emulating (time, position) available on other platforms!
                    float v = (positions[i] - c)/(times[i] - currenttime);
                    m_servos[i]->setPosition(positions[i]);
                    m_servos[i]->setVelocity(fabs(v*1000));
                    m_servos[i]->setControlP(gains[i]);
                }
            }
        }
    }
    if (m_data->getNextJointTorques(isvalid, times, torques, gains))
    {
        for (int i=0; i<m_servos.size(); i++)
        {
            if (isvalid[i] == true)
                m_servos[i]->setForce(torques[i]);
        }
    }
    if (m_data->getNextLeds(isvalid, times, redvalues, greenvalues, bluevalues))
    {
        for (int i=0; i<m_leds.size(); i++)
        {
            if (isvalid[i] == true)
            {
                int ledvalue = (255*((int) redvalues[i]) << 4) + (255*((int) greenvalues[i]) << 2) + (255*((int) bluevalues[i]));       // convert to hex: RRGGBB
                m_leds[i]->set(ledvalue);
            }
        }
    }
#if DEBUG_NUACTIONATORS_VERBOSITY > 6
    m_data->summaryTo(debug);
#endif
}




