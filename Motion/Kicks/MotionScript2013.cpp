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

void MotionScript2013::ApplyCurrentFrameToRobot(m_actions)
{
    auto& current_frame = script_frames_[current_frame_index_];

    ++m_current_index;
    std::vector<float> joints = m_joints.at(m_current_index);
    std::vector<float> gains = m_gains.at(m_current_index);

    float targetTime = m_script_start_time + m_current_script->m_times.back().at(m_current_index);

    m_actions->add(NUActionatorsData::Head, targetTime, nu_nextHeadJoints, nu_nextHeadGains);
    m_actions->add(NUActionatorsData::LArm, targetTime, nu_nextLeftArmJoints, nu_nextLeftArmGains);
    m_actions->add(NUActionatorsData::RArm, targetTime, nu_nextRightArmJoints, nu_nextRightArmGains);
    m_actions->add(NUActionatorsData::LLeg, targetTime, nu_nextLeftLegJoints, nu_nextLeftLegGains);
    m_actions->add(NUActionatorsData::RLeg, targetTime, nu_nextRightLegJoints, nu_nextRightLegGains);
}

NUData::id_t 

void MotionScript2013::Reset()
{
    current_frame_index_ = 0;
}
