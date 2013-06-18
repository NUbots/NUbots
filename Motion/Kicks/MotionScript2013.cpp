#include <string>
#include <stdexcept>
#include <unordered_map>
#include <iostream>
#include "MotionScript2013.h"
#include "Infrastructure/NUData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Framework/darwin/Framework/include/JointData.h"
#include "Motion/Tools/MotionFileTools.h"
#include "debug.h"
#include "Infrastructure/NUBlackboard.h"
#include "ConfigSystem/ConfigManager.h"


NUData::id_t MotionScriptFrame::MapServoIdToNUDataId(int sensor_id)
{
    switch(sensor_id)
    {
        case Robot::JointData::ID_R_SHOULDER_PITCH: return NUData::RShoulderPitch;
        case Robot::JointData::ID_L_SHOULDER_PITCH: return NUData::LShoulderPitch;
        case Robot::JointData::ID_R_SHOULDER_ROLL:  return NUData::RShoulderRoll;
        case Robot::JointData::ID_L_SHOULDER_ROLL:  return NUData::LShoulderRoll;
        case Robot::JointData::ID_R_ELBOW:          return NUData::RElbowPitch;
        case Robot::JointData::ID_L_ELBOW:          return NUData::LElbowPitch;
        case Robot::JointData::ID_R_HIP_YAW:        return NUData::RHipYaw;
        case Robot::JointData::ID_L_HIP_YAW:        return NUData::LHipYaw;
        case Robot::JointData::ID_R_HIP_ROLL:       return NUData::RHipRoll;
        case Robot::JointData::ID_L_HIP_ROLL:       return NUData::LHipRoll;
        case Robot::JointData::ID_R_HIP_PITCH:      return NUData::RHipPitch;
        case Robot::JointData::ID_L_HIP_PITCH:      return NUData::LHipPitch;
        case Robot::JointData::ID_R_KNEE:           return NUData::RKneePitch;
        case Robot::JointData::ID_L_KNEE:           return NUData::LKneePitch;
        case Robot::JointData::ID_R_ANKLE_PITCH:    return NUData::RAnklePitch;
        case Robot::JointData::ID_L_ANKLE_PITCH:    return NUData::LAnklePitch;
        case Robot::JointData::ID_R_ANKLE_ROLL:     return NUData::RAnkleRoll;
        case Robot::JointData::ID_L_ANKLE_ROLL:     return NUData::LAnkleRoll;
        case Robot::JointData::ID_HEAD_PAN:         return NUData::HeadYaw;
        case Robot::JointData::ID_HEAD_TILT:        return NUData::HeadPitch;
        default: {
            std::cout << __PRETTY_FUNCTION__ 
                      << " - Invalid sensor_id: " << sensor_id << ";";
            return NUData::id_t();
        }
    }
}

int MapRowIndexToServoId(int index)
{
    switch(index)
    {
         case( 0): return Robot::JointData::ID_HEAD_TILT;        
         case( 1): return Robot::JointData::ID_HEAD_PAN;         
         case( 2): return Robot::JointData::ID_L_SHOULDER_ROLL;  
         case( 3): return Robot::JointData::ID_L_SHOULDER_PITCH; 
         case( 4): return Robot::JointData::ID_L_ELBOW;          
         case( 5): return Robot::JointData::ID_R_SHOULDER_ROLL;  
         case( 6): return Robot::JointData::ID_R_SHOULDER_PITCH; 
         case( 7): return Robot::JointData::ID_R_ELBOW;          
         case( 8): return Robot::JointData::ID_L_HIP_ROLL;       
         case( 9): return Robot::JointData::ID_L_HIP_PITCH;      
         case(10): return Robot::JointData::ID_L_HIP_YAW;        
         case(11): return Robot::JointData::ID_L_KNEE;           
         case(12): return Robot::JointData::ID_L_ANKLE_ROLL;     
         case(13): return Robot::JointData::ID_L_ANKLE_PITCH;    
         case(14): return Robot::JointData::ID_R_HIP_ROLL;       
         case(15): return Robot::JointData::ID_R_HIP_PITCH;      
         case(16): return Robot::JointData::ID_R_HIP_YAW;        
         case(17): return Robot::JointData::ID_R_KNEE;           
         case(18): return Robot::JointData::ID_R_ANKLE_ROLL;     
         case(19): return Robot::JointData::ID_R_ANKLE_PITCH;    
        default: {
            std::cout   << __PRETTY_FUNCTION__
                        << ": Invalid index: " << sensor_id << ";"
                        << std::endl;
            return -1;
        }
    }
}

void MotionScriptFrame::ApplyToRobot(NUActionatorsData* actionators_data)
{
    for(auto key_value : joints_)
    {
        auto& joint = key_value.second;

        if(!joint.GetDisable())
        {
            actionators_data->add(
                MapServoIdToNUDataId(joint.GetServoId()),
                duration_,
                joint.GetPosition(),
                joint.GetGain());
        }
    }
}

void MotionScriptFrame::AddDescriptor(int servo_id, ScriptJointDescriptor descriptor)
{
    joints_[servo_id] = descriptor;
}

void MotionScriptFrame::DeleteDescriptor(int servo_id)
{
    joints_.erase(servo_id);
}

bool MotionScriptFrame::GetDescriptor(int servo_id, ScriptJointDescriptor* descriptor)
{
    if(descriptor == nullptr)
        return nullptr;

    try
    {
        *descriptor =  joints_.at(servo_id);
        return true;
    }
    catch(std::out_of_range)
    {
        return false;
    }
}


MotionScript2013::~MotionScript2013()
{
    for(MotionScriptFrame* frame : script_frames_)
        delete frame;
}

MotionScript2013* MotionScript2013::LoadFromConfigSystem(
    const std::string& path)
{
    /* Format:
    'path.to.script': {
        'num_frames': 2,
        'frame_0': {
            'time_': '1.5',
            'servo_id_3': {
                'position_': 1.0,
                'gain_': 1.0,
            }
        },
        'frame_1': {
            'time_': '3.8',
            'servo_id_5': {
                'position_': 2.0,
                'gain_': 32.0,
            },
            'servo_id_19': {
                'position_': -1.5,
                'gain_': 0.7,
            }
        }
    }
    */

    ConfigSystem::ConfigManager* config = Blackboard->Config;

    bool success;

    int num_frames;
    // success = config->ReadValue<int>(path, "num_frames", &num_frames);
    config->ReadValue(path, path, nullptr);

    if(!success)
        return nullptr;

    auto* script = new MotionScript2013();

    for(int i = 0; i < num_frames; i++)
    {
        auto* frame = new MotionScriptFrame();

        script->AddFrame(frame);
    }
    
    return nullptr;
}

static MotionScript2013* MotionScript2013::LoadOldScript(const std::string& path){
    //Method modified from Motion/Tools/MotionScript.h
    std::ifstream file(path);
    if (!file.is_open())
    {
        errorlog << "MotionScript2013::LoadOldScript(). Unable to open. " << path << std::endl;
        return false;
    }
    else
    {
        MotionFileTools::toFloat(file);
        MotionFileTools::toBool(file);
        labels = MotionFileTools::toStringVector(file);
        if (labels.empty())
        {
            errorlog << "MotionScript::load(). Unable to load " << path << " the file labels are invalid " << std::endl;
            return false;
        }
        playspeed = 1.0;
        
        int numjoints = labels.size() - 1;
        times = vector<vector<double> >(numjoints, vector<double>());
        positions = vector<vector<float> >(numjoints, vector<float>());
        gains = vector<vector<float> >(numjoints, vector<float>());
        
        float time;
        vector<vector<float> > row;
        
        while (!file.eof())
        {
            MotionFileTools::toFloatWithMatrix(file, time, row);
            if (row.size() >= numjoints)
            {   // discard rows that don't have enough joints
                for (int i=0; i<numjoints; i++)
                {
                    if (row[i].size() > 0)
                    {   // if there is an entry then the first must be a position
                        times[i].push_back(1000*time);
                        positions[i].push_back(row[i][0]);
                        
                        // because the way the joint actionators are designed we must also specify a gain
                        if (row[i].size() > 1)
                        {   // if there is a second entry then it is the gain
                            gains[i].push_back(row[i][1]);
                        }
                        else
                        {   // however if there is no second entry we reuse the previous entry or use 100% if this is the first one
                            if (gains[i].empty())
                                gains[i].push_back(100.0);
                            else
                                gains[i].push_back(gains[i].back());
                        }
                        
                    }
                }
            }
            row.clear();
        }
        file.close();        

        return InitialiseScriptFromOld(times,positions,gains);
    }

}

MotionScript2013* MotionScript2013::InitialiseScriptFromOld(vector<float> times, vector<vector<float> > positions, vector<vector<float> > gains){
    
    MotionScript2013* script = new MotionScript2013();
    for(int frame_number = 0; frame_number<times.size();frame_number++){

        MotionScriptFrame new_frame();

        for(int motor_index; motor_index<positions[frame_number].size();motor_index++){

            ScriptJointDescriptor descriptor();
            descriptor.SetServoId(MapRowIndexToServoId(motor_index));
            descriptor.SetPosition(MapRowIndexToServoId(motor_index),positions[frame_number][motor_index]);
            descriptor.SetPosition(MapRowIndexToServoId(motor_index),gains[frame_number][motor_index]);
            new_frame.AddDescriptor(MapRowIndexToServoId(motor_index),descriptor);

        }
        script->AddFrame(&new_frame);
    }
    return script;

}

bool MotionScript2013::SaveToConfigSystem(
    const MotionScript2013& script,
    const std::string& path)
{
    return false;
}

void MotionScript2013::AdvanceToNextFrame()
{
    current_frame_index_++;
}

void MotionScript2013::SeekFrame(int frame)
{
    if(frame < 0 || frame >= script_frames_.size())
    {
        std::cout << "This script does not contain the frame: " 
                  << frame << ";" << std::endl;
        return;
    }
    current_frame_index_ = frame;
}

void MotionScript2013::ApplyCurrentFrameToRobot(NUActionatorsData* actionators_data)
{
    auto current_frame = script_frames_[current_frame_index_];

    current_frame->ApplyToRobot(actionators_data);
}


void MotionScript2013::Reset()
{
    current_frame_index_ = 0;
}

void MotionScript2013::StartScript(NUActionatorsData* actionators_data)
{
    kick_enable_time_ = actionators_data->CurrentTime;
}

bool MotionScript2013::HasCompleted(float current_time)
{
    if(current_frame_index_ < script_frames_.size())
        return false;

    float script_time = current_time - script_start_time_;
    auto& current_frame = script_frames_[current_frame_index_];

    if(script_time < GetScriptDuration())
        return false;

    return true;
}

float MotionScript2013::GetNextFrameTime(float current_time)
{
    return current_time + script_frames_[current_frame_index_]->GetDuration();
}

int MotionScript2013::GetFrameCount()
{
    return script_frames_.size();
}

int MotionScript2013::GetCurrentFrameIndex()
{
    return current_frame_index_;
}

MotionScriptFrame* MotionScript2013::GetCurrentFrame()
{
    return script_frames_[current_frame_index_];
}

void MotionScript2013::InsertFrame(int index, MotionScriptFrame* frame)
{
    auto it = script_frames_.begin();

    script_frames_.insert(it + index, frame);
}

void MotionScript2013::RemoveFrame(int index)
{
    auto it = script_frames_.begin();

    script_frames_.erase(it + index);
}

void MotionScript2013::DuplicateFrame(int index)
{
    auto* current_frame = script_frames_[current_frame_index_];

    auto* new_frame = new MotionScriptFrame(*current_frame);

    InsertFrame(index, current_frame);
}

float MotionScript2013::GetScriptDuration()
{
    float script_duration = 0;

    for(auto* frame : script_frames_)
        script_duration += frame->GetDuration();

    return script_duration;
}

void MotionScript2013::AddFrame(MotionScriptFrame* frame)
{
    script_frames_.push_back(frame);
}
