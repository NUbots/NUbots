/*!  @file ScriptTunerStates.h
    @brief Provider of Head behaviour Testing behaviour. Darwin simply stands, observes and localises. Modified from Zombie state.
    @author Jake Fountain

    @author Jake Fountain

    Copyright (c) 2012 Jake Fountain

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

#ifndef SCRIPT_TUNER_STATES_H
#define SCRIPT_TUNER_STATES_H

#include "Behaviour/BehaviourState.h"
#include "ScriptTunerProvider.h"

#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/Jobs/MotionJobs/KickJob.h"
#include "Infrastructure/Jobs/MotionJobs/WalkJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadTrackJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadPanJob.h"
#include "Infrastructure/Jobs/MotionJobs/HeadNodJob.h"
#include "Infrastructure/Jobs/MotionJobs/MotionFreezeJob.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include <thread>
#include <chrono>
#include "Framework/darwin/Framework/include/JointData.h"
#include "Motion/Kicks/MotionScript2013.h"
#include "Infrastructure/NUData.h"

#include "Autoconfig/nubotdataconfig.h"

#include "Behaviour/Common/HeadBehaviour.h"


#include "debug.h"

using std::vector;
using std::string;

class ScriptTunerCommand
{
public:
    enum class CommandType {
        kUnknown,       //!< Unknown command.
        kHelp,          //!< Prints help for the ScriptTuner.
        kExit,          //!< Exit the script tuner.
        kPlay,          //!< Plays the current script.
        kEdit,          //!< Begin editing the current sctipr.
        kNextFrame,     //!< Move to the next frame.
        kNewFrame,      //!< Creates a new frame after the current frame.
        kFrameSeek,     //!< Seek to a given frame.
        kFrameDuration, //!< Set the duration of the current frame.
        kAllOn,         //!< Turn on all motors.
        kJointPosition, //!< Modify the position of a given joint.
        kJointGain,     //!< Modify the gain of a given joint.
        kJointPositionGain,     //!< Modify the gain of a given joint.
        kJointOff,      //!< Temporarily turn off a given joint, for editing.
        kJointOn,       //!< Turn on a given joint.
        kSaveFrame,     //!< Save the current frame.
        kSaveScript,    //!< Save the current script.
    };

    static int MapJointInitialismToServoId(const std::string& initials)
    {
        if(!initials.compare("RSP")) return Robot::JointData::ID_R_SHOULDER_PITCH;
        if(!initials.compare("LSP")) return Robot::JointData::ID_L_SHOULDER_PITCH;
        if(!initials.compare("RSR")) return Robot::JointData::ID_R_SHOULDER_ROLL;
        if(!initials.compare("LSR")) return Robot::JointData::ID_L_SHOULDER_ROLL;
        if(!initials.compare("RE" )) return Robot::JointData::ID_R_ELBOW;
        if(!initials.compare("LE" )) return Robot::JointData::ID_L_ELBOW;
        if(!initials.compare("RHY")) return Robot::JointData::ID_R_HIP_YAW;
        if(!initials.compare("LHY")) return Robot::JointData::ID_L_HIP_YAW;
        if(!initials.compare("RHR")) return Robot::JointData::ID_R_HIP_ROLL;
        if(!initials.compare("LHR")) return Robot::JointData::ID_L_HIP_ROLL;
        if(!initials.compare("RHP")) return Robot::JointData::ID_R_HIP_PITCH;
        if(!initials.compare("LHP")) return Robot::JointData::ID_L_HIP_PITCH;
        if(!initials.compare("RK" )) return Robot::JointData::ID_R_KNEE;
        if(!initials.compare("LK" )) return Robot::JointData::ID_L_KNEE;
        if(!initials.compare("RAP")) return Robot::JointData::ID_R_ANKLE_PITCH;
        if(!initials.compare("LAP")) return Robot::JointData::ID_L_ANKLE_PITCH;
        if(!initials.compare("RAR")) return Robot::JointData::ID_R_ANKLE_ROLL;
        if(!initials.compare("LAR")) return Robot::JointData::ID_L_ANKLE_ROLL;
        if(!initials.compare("HP" )) return Robot::JointData::ID_HEAD_PAN;
        if(!initials.compare("HT" )) return Robot::JointData::ID_HEAD_TILT;
        return Robot::JointData::ID_UNKNOWN;
    }

    static ScriptTunerCommand ParseCommand(const std::string& command)
    {
        std::stringstream command_ss;
        command_ss << command;

        std::string arg0;
        command_ss >> arg0;

        if(!arg0.compare("help")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kHelp);
            return c;
        } else if(!arg0.compare("play")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kPlay);
            return c;
        } else if(!arg0.compare("edit")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kEdit);
            return c;
        } else if(!arg0.compare("exit")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kExit);
            return c;
        } else if(!arg0.compare("quit")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kExit);
            return c;
        } else if(!arg0.compare("saveframe")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kSaveFrame);
            return c;
        } else if(!arg0.compare("savescript")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kSaveScript);
            return c;
        } else if(!arg0.compare("newframe")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kNewFrame);
            return c;
        } else if(!arg0.compare("seek")) {
            int frame_number;
            if(!command_ss >> frame_number)
                return false;
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kFrameSeek);
            c.set_frame_number(frame_number);
            return c;
        } else if(!arg0.compare("duration")) {
            float duration;
            if(!command_ss >> duration)
                return false;
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kFrameDuration);
            c.set_duration(duration);
            return c;
        } else if(!arg0.compare("allon")) {
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kAllOn);
            return c;
        } else if(!arg0.compare("on")) {
            int joint_id;
            if(!command_ss >> joint_id)
                return false;
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kJointOn);
            c.set_joint_number(joint_id);
            return c;
        } else if(!arg0.compare("off")) {
            int joint_id;
            if(!command_ss >> joint_id)
                return false;
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kJointOff);
            c.set_joint_number(joint_id);
            return c;
        } else {
            float position;
            float duration;
            bool set_duration = false;
            if(!command_ss >> position)
                return false;
            if(command_ss >> duration)
                set_duration = true;
            int joint_id = MapJointInitialismToServoId(arg0);
            ScriptTunerCommand c;
            c.set_command_type(CommandType::kJointPositionGain);
            c.set_joint_number(joint_id);
            c.set_position(position);
            if(set_duration)
                c.set_duration(duration);
            return c;
        }
    }

    static void PrintCommandsLongHelp()
    {
        std::cout << std::endl;
        std::cout << "Script Tuner Command Help:" << std::endl;
        std::cout << std::endl;
        std::cout << "Note: When a command requires a joint to be specified, " << std::endl;
        std::cout << "      it should be denoted using one of the following initialisms:" << std::endl;
        std::cout << std::endl;
        std::cout << "RSP = Right Shoulder Pitch;" << std::endl;
        std::cout << "LSP = Left Shoulder Pitch;" << std::endl;
        std::cout << "RSR = Right Shoulder Roll;" << std::endl;
        std::cout << "LSR = Left Shoulder Roll;" << std::endl;
        std::cout << "RE  = Right Elbow;" << std::endl;
        std::cout << "LE  = Left Elbow;" << std::endl;
        std::cout << "RHY = Right Hip Yaw;" << std::endl;
        std::cout << "LHY = Left Hip Yaw;" << std::endl;
        std::cout << "RHR = Right Hip Roll;" << std::endl;
        std::cout << "LHR = Left Hip Roll;" << std::endl;
        std::cout << "RHP = Right Hip Pitch;" << std::endl;
        std::cout << "LHP = Left Hip Pitch;" << std::endl;
        std::cout << "RK  = Right Knee;" << std::endl;
        std::cout << "LK  = Left Knee;" << std::endl;
        std::cout << "RAP = Right Ankle Pitch;" << std::endl;
        std::cout << "LAP = Left Ankle Pitch;" << std::endl;
        std::cout << "RAR = Right Ankle Roll;" << std::endl;
        std::cout << "LAR = Left Ankle Roll;" << std::endl;
        std::cout << "HP  = Head Pan;" << std::endl;
        std::cout << "HT  = Head Tilt;" << std::endl;
    }

    ScriptTunerCommand() : command_type_(CommandType::kUnknown),
                           position_(0),
                           gain_(0),
                           duration_(1000),
                           disable_(false),
                           joint_number_(0),
                           frame_number_(0) {}

    CommandType command_type() { return command_type_; }
    float position() { return position_; }
    float gain() { return gain_; }
    float duration() { return duration_; }
    bool disable() { return disable_; }
    int joint_number() { return joint_number_; }
    int frame_number() { return frame_number_; }

    void set_command_type(CommandType command_type) { command_type_ = command_type; }
    void set_position(float position) { position_ = position; }
    void set_gain(float gain) { gain_ = gain; }
    void set_duration(float duration) { duration_ = duration; }
    void set_disable(bool disable) { disable_ = disable; }
    void set_joint_number(int joint_number) { joint_number_ = joint_number; }
    void set_frame_number(int frame_number) { frame_number_ = frame_number; }

private:
    CommandType command_type_;
    float position_;
    float gain_;
    float duration_;
    bool disable_;
    int joint_number_;
    int frame_number_;
};

class ScriptTunerSubState : public BehaviourState
{
public:
    ScriptTunerSubState(ScriptTunerProvider* provider){m_provider = provider;}
protected:
    ScriptTunerProvider* m_provider;
};

// ----------------------------------------------------------------------------------------------------------------------- PausedState
class ScriptTunerState : public ScriptTunerSubState
{
public:
    std::map<string, int> string_id_to_int_id; 
    string m_file_name;
    string m_file_path;

    MotionScript2013* script;

    bool m_script_active;

    vector<int> motors_to_be_saved;//Contains motors which have had torque turned off and then on.
    

    NUActionatorsData* m_actionators_data;
    NUSensorsData* m_sensors_data;

    ScriptTunerState(ScriptTunerProvider* provider);

    BehaviourState* nextState() {return m_provider->m_state;}
    void doState();

    void editCurrentFrame();

    /*! @brief Loads a motion script from a file into the member variables frames and times */
    bool loadScript(string filename);

     /*! @brief Applies frame, given by frames[frame], to robot.*/
    void applyFrameToRobot();

     /*! @brief Reads motor positions from robot and saves them to the current frame, frames[frame].
        Only save motor positions which have had torque off and then back on.*/
    void saveManuallyMovedMotors();

     /*! @brief Writes the frames to script file.*/
    bool saveScriptToFile(string filename);

     /*! @brief Adds a new frame to the vector of frames.*/
    void addFrame(string argument);

    /*! @brief Moves to the frame indicated by the first argument of command string, if it is an integer.*/
    void interpretSeekCommand(string command);

    /*! @brief Moves script to a given frame if it is a valid frame number.*/
    void moveToFrame(int frame_number);

    /*! @brief Exit program and load a new script.*/
    void exitScript();

    /*! @brief Play the script from the current frame.*/
    void playScript();

    /*! @brief Applies positional or torque settings to motors.
        @param other_parameters = "<position> <gain>". If gain is not specified a default value is used.*/
    void applyRequestToMotors(string parameters);
    
    /*! @brief Motor fine tuning.
        @param pos_change is change in angle in radians.*/
    void changeMotorPosition(int motor_id,float pos_change);
    
    /*! @brief Motor fine tuning.
        @param gain_change is change in gain.*/    
    void changeMotorGain(int motor_id, float gain_change);
    
    /*! @brief Motor turn off torque method.*/    
    void turnOffMotor(int motor_id);
    
    /*! @brief Motor turn on torque method.
         Also marks motor for saving with save manually moved motors.
         Adds motion job for motor at current position with gain given in script.*/    
    void turnOnMotor(int motor_id);
    
    /*! @brief Returns the current frame number.
    */
    int getCurrentFrameNumber();
    
    /*! @brief Returns the total number of frames for the loaded script.
    */    
    int totalNumberOfFrames();
    
    /*! @brief Returns the time required for the current frame to complete.
    */    
    float durationOfCurrentFrame();

    /*! @brief Indicates state of script during editing.
    */
    bool scriptIsActive();
    
    /*! @brief Updates robot to have motor positions indicated by the current frame in the current script.
    */
    void applyCurrentFrameToRobot();

    /*! @brief Sets the frame duration.
    */
    void setCurrentFrameDuration(string duration_string);

    /*! @brief Returns the MEASURED motor position.
    */
    float getMotorPosition(int motor_id);

    /*! @brief Tests whether the motor torque is on.
    */
    bool motorTorqueIsOff(int motor_id);

    /*! @brief Turns on all motors and marks those without torque for saving.
    */
    void turnOnAllMotors();

};

#endif

