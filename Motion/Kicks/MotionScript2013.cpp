#include "MotionScript2013.h"
#include "NUData.h"

MotionScript2013::MotionScript2013()
{

}

MotionScript2013::~MotionScript2013()
{
    for(MotionScriptFrame* frame : script_frames_)
        delete frame;
}

MotionScript2013::MotionScript2013* LoadFromConfigSystem(
    const std::string& path,
    const std::string& name)
{
    return nullptr;
}

bool MotionScript2013::SaveToConfigSystem(
    const MotionScript& script,
    const std::string& path,
    const std::string& name)
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

    current_frame.ApplyToRobot(script_start_time_, actionators_data);
}


void MotionScript2013::Reset()
{
    current_frame_index_ = 0;
}

bool MotionScript2013::HasCompleted(float current_time)
{
    if(current_frame_index_ < script_frames_.size())
        return false;

    float script_time = current_time - script_start_time_;
    auto& current_frame = script_frames_[current_frame_index_];

    if(script_time < current_frame.GetTime())
        return false;

    return true;
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
            return id_t();
        }
    }
}

void MotionScriptFrame::ApplyToRobot(float script_start_time, NUActionatorsData* actionators_data)
{
    auto target_time = script_start_time + _time;

    for(ScriptJointDescriptor* joint : joints_)
    {
        m_actions->add(
            MapServoIdToNUDataId(joint->GetServoId(),
            target_time,
            joint->GetPosition(),
            joint->GetGain());
    }
}
