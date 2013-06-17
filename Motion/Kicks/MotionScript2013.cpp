#include <stdexcept>
#include <unordered_map>
#include <iostream>
#include "MotionScript2013.h"
#include "Infrastructure/NUData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Framework/darwin/Framework/include/JointData.h"

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
    
    
    
    return nullptr;
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
    auto& current_frame = script_frames_[current_frame_index_];

    current_frame->ApplyToRobot(script_start_time_, actionators_data);
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

    if(script_time < current_frame->GetTime())
        return false;

    return true;
}

float MotionScript2013::GetNextFrameTime(float current_time)
{
    if(current_frame_index_ == GetFrameCount() - 1)
        return script_frames_[current_frame_index_]->GetTime();
    else
        return script_frames_[current_frame_index_ + 1]->GetTime();
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

void MotionScriptFrame::ApplyToRobot(float script_start_time, NUActionatorsData* actionators_data)
{
    auto target_time = script_start_time + time_;

    for(auto key_value : joints_)
    {
        auto& joint = key_value.second;

        if(!joint.GetDisable())
        {
            actionators_data->add(
                MapServoIdToNUDataId(joint.GetServoId()),
                target_time,
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

