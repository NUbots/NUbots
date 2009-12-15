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

// init m_camera_setting_names:
static string temp_setting_names[] = {string("SelectCamera")};
vector<string> NAOWebotsActionators::m_camera_setting_names(temp_setting_names, temp_setting_names + sizeof(temp_setting_names)/sizeof(*temp_setting_names));

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
    m_current_time = 0;
    getActionatorsFromWebots(platform);
    enableActionatorsInWebots();
    
    m_data->setAvailableJointControlMethods(m_servo_control_names);
    m_data->setAvailableJoints(m_servo_names);
    m_data->setAvailableLeds(m_led_names);
    m_data->setAvailableCameraSettings(m_camera_setting_names);
    //m_data->setAvailableOtherActionators();      there are no other actionators at the moment 
    
    
    m_data->addJointPosition(NUActionatorsData::HeadYaw, platform->system->getTime() + 350, 0, 1, 30);
    m_data->addJointPosition(NUActionatorsData::HeadYaw, platform->system->getTime() + 4000, -1.57, 1, 30);
    m_data->addJointPosition(NUActionatorsData::HeadYaw, platform->system->getTime() + 8000, 1.57, 1, 30);
    
    vector<float> pos (2, 0);
    vector<float> vel (2, 1);
    vector<float> gain (2, 100);
    pos[1] = -0.7;
    m_data->addJointPositions(NUActionatorsData::Head, platform->system->getTime() + 10000, pos, vel, gain);
    
    // I am temporarily enabling the camera here because it doesn't appear in the simulation unless it is enabled!
    Camera* camera = m_platform->getCamera("camera");
    camera->enable(80);         // the timestep for the camera has to be a multiple of 40ms, so the possible frame rates are 25, 12.5, 8.33 etc
    
    vector<float> data (1,0);
    data[0] = 0;
    m_data->addCameraSetting(NUActionatorsData::SelectCamera, platform->system->getTime() + 5000, data);
    data[0] = 1;
    m_data->addCameraSetting(NUActionatorsData::SelectCamera, platform->system->getTime() + 10000, data);
    
    
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
    m_camera_select = platform->getServo("CameraSelect");
    // Get the leds
    for (int i=0; i<m_led_names.size(); i++)
        m_leds.push_back(platform->getLED(m_led_names[i]));
    //! @todo TODO: get the sound from webots
}

/*! @brief enable the actionators in the simulated NAO
 */
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
    
    m_current_time = m_platform->system->getTime();
    m_data->removeCompletedPoints(m_current_time);
    
    copyToServos();
    copyToCamera();
    copyToLeds();
    copyToSound();
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 6
    m_data->summaryTo(debug);
#endif
}

/*! @brief Copies the joint positions and torques to the servos
 */
void NAOWebotsActionators::copyToServos()
{
    static vector<bool> isvalid;            
    static vector<double> times;
    static vector<float> positions;
    static vector<float> velocities;
    static vector<float> torques;
    static vector<float> gains;
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::copyToServos()" << endl;
#endif
    
    // Positions
    if (m_data->getNextJointPositions(isvalid, times, positions, velocities, gains))
    {
        if (m_servos.size() == isvalid.size())                          // only process the input if it has the right length
        {
            for (int i=0; i<m_servos.size(); i++)
            {
                if (isvalid[i] == true)
                {
                    if ((times[i] - m_current_time) > m_simulation_step)
                    {
                        float c = m_servos[i]->getPosition();           // i think I am allowed to do this right? I ought to be I am only emulating (time, position) available on other platforms!
                        float v = (positions[i] - c)/(times[i] - m_current_time);
                        m_servos[i]->setControlP(gains[i]/10.0);
                        m_servos[i]->setVelocity(fabs(v*1000));
                        m_servos[i]->setPosition(positions[i]);
                    }
                }
            }
        }
        else
            debug << "NAOWebotsActionators::copyToServos(). The input does not have the correct length, all data will be ignored!" << endl;

    }
    // Torques
    if (m_data->getNextJointTorques(isvalid, times, torques, gains))
    {
        if (m_servos.size() == isvalid.size())
        {
            for (int i=0; i<m_servos.size(); i++)
            {
                if (isvalid[i] == true)
                    m_servos[i]->setForce(torques[i]);          // I think this is broken in webots!
            }
        }
        else
            debug << "NAOWebotsActionators::copyToServos(). The input does not have the correct length, all data will be ignored!" << endl;
    }
}

/*! @brief Copies the camera settings to the camera
 */
void NAOWebotsActionators::copyToCamera()
{
    static vector<bool> isvalid;
    static vector<double> times;
    static vector<vector<float> > data;
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::copyToCamera()" << endl;
#endif
    
    if (m_data->getNextCameraSettings(isvalid, times, data))
    {
        if (isvalid[0] == true)
        {
            if (data[0].size() > 0)
            {
                if (data[0][0] == 0)
                    m_camera_select->setPosition(0);
                else
                    m_camera_select->setPosition(0.6981);       // offset between top and bottom camera is 40 degrees = 0.6981
            }
        }
    }
}

/*! @brief Copies the led values to the leds
 */
void NAOWebotsActionators::copyToLeds()
{
    static vector<bool> isvalid;
    static vector<double> times;
    static vector<float> redvalues;
    static vector<float> greenvalues;
    static vector<float> bluevalues;
    
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::copyToLeds()" << endl;
#endif
    
    if (m_data->getNextLeds(isvalid, times, redvalues, greenvalues, bluevalues))
    {
        if (m_leds.size() == isvalid.size())
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
        else
            debug << "NAOWebotsActionators::copyToLeds(). The input does not have the correct length, all data will be ignored!" << endl;
    }
}

/*! @brief Copies the sound to the sound driver
 */
void NAOWebotsActionators::copyToSound()
{
#if DEBUG_NUACTIONATORS_VERBOSITY > 4
    debug << "NAOWebotsActionators::copyToSound()" << endl;
#endif
}



