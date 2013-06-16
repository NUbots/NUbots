#include "ScriptKick2013.h"
#include "debug.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Motion/NUWalk.h"

//for configs
#include "nubotdataconfig.h"
#include <boost/algorithm/string.hpp>


ScriptKick2013(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUKick(walk, data, actions)
{
    // Load scripts for all kick types:
    const std::string script_path = "";
    left_kick_script_       = MotionScript2013.LoadFromConfigSystem(script_path, "left");
    right_kick_script_      = MotionScript2013.LoadFromConfigSystem(script_path, "right");
    side_left_kick_script_  = MotionScript2013.LoadFromConfigSystem(script_path, "side_left");
    side_right_kick_script_ = MotionScript2013.LoadFromConfigSystem(script_path, "side_right");

    // Set to null while not kicking
    current_script_ = nullptr;
}

~ScriptKick2013()
{
    // Delete all scripts:
    delete left_kick_script_;
    delete right_kick_script_;
    delete side_left_kick_script_;
    delete side_right_kick_script_;
}

void ScriptKick2013::kickToPoint(const std::vector<float>& position, const std::vector<float>& target)
{
    // Ignore calls to this method while a kick is in progress
    if(isActive())
        return;

    float ball_x = position[0];
    float ball_y = position[1];
    float target_x = target[0];
    float target_y = target[1];
    float theta = atan2f(target_y - ball_y, target_x - ball_x);

    // triggers sidekick too often with -45 deg to 45 deg front kick zone
    float angle_margin = mathGeneral::PI / 4.0f; 


    if((side_left_kick_script_ != nullptr
        && theta > angle_margin 
        && side_left_kick_area_.PointInside(ball_x, ball_y)) 
        ) {
        StartKick(side_left_kick_script_, rightLeg);
    } else if(left_kick_script_ != nullptr
              && theta <= angle_margin 
              && theta >= -angle_margin
              && left_kick_area_.PointInside(ball_x, ball_y)
              ) {
        StartKick(left_kick_script_, leftLeg);
    } else if(side_right_kick_script_ != nullptr
              && theta < -angle_margin
              && side_right_kick_area_.PointInside(ball_x, ball_y)
              ) {
        StartKick(side_right_kick_script_, leftLeg);
    } else if(right_kick_script_ != nullptr
              && theta >= -angle_margin
              && theta <= angle_margin
              && right_kick_area_.PointInside(ball_x, ball_y)
              ) {
        StartKick(right_kick_script_, rightLeg);
    } else {
        //std::cout << "No kick available for position: (" << ball_x << ", " << ball_y << ")" << std::endl;
        return;
    }
}

void ScriptKick2013::StartKick(
    MotionScript* kick_script, 
    KickingLeg kicking_leg)
{
    m_kicking_leg = kicking_leg;
    current_script_ = kick_script;

    current_kick_->Reset();
    m_kick_enable_time = m_data->CurrentTime;

    m_walk->stop();
    setArmEnabled(true, true);
    setHeadEnabled(true);
    m_kick_ready = true;
    m_kick_enabled = true;
}

void ScriptKick2013::doKick()
{
    // If doKick is called while the robot is not kicking, just return.
    if(!isActive())
        return;

    // If the current script is complete
    if(current_script_->HasCompleted())
    {
        kill();
        return;
    }

    // If it's time for the next frame
    if(GetCurrentScriptTime() >= current_script_->GetNextFrameTime())
    {
        // Schedule the next joint positions
        current_script_->AdvanceToNextFrame();
        current_script_->ApplyCurrentFrameToRobot(m_actions);
    }
}

void ScriptKick2013::stop() { kill(); }
void ScriptKick2013::kill() 
{ 
    current_script_ = nullptr;
    m_kick_ready = false;
    m_kick_enabled = false;
    setArmEnabled(false, false);
    setHeadEnabled(false);
    m_kicking_leg = noLeg;
}

bool isActive()
{
    return current_script_ != nullptr;
}

bool ScriptKick2013::isUsingHead() { return false; }
bool ScriptKick2013::isUsingArms() { return true; }
bool ScriptKick2013::isUsingLegs() { return true; }
bool ScriptKick2013::requiresHead() { return false; }
bool ScriptKick2013::requiresArms() { return true; }
bool ScriptKick2013::requiresLegs() { return true; }

float ScriptKick2013::GetCurrentScriptTime()
{
    return m_data->CurrentTime;
}
