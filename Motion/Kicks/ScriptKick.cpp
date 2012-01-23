#include "ScriptKick.h"
#include "Motion/Tools/MotionScript.h"

ScriptKick::ScriptKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions): NUKick(walk, data, actions)
{
    m_left_kick_script = MotionScript("LeftKick");
    m_right_kick_script = MotionScript("RightKick");
    m_current_script = NULL;
    m_script_start_time = -1;
}

ScriptKick::~ScriptKick()
{
    delete m_left_kick_script;
    delete m_right_kick_script;
    m_current_script = NULL;
}

bool ScriptKick::isActive()
{
    if(m_current_script != NULL)
    {
        return m_data->CurrentTime <= m_current_script->timeFinished() + m_script_start_time;
    }
    else
    {
        return false;
    }
}

bool ScriptKick::isUsingHead()
{
    if(m_current_script != NULL)
    {
        return m_data->CurrentTime <= m_current_script->timeFinishedWithHead() + m_script_start_time;
    }
    else
    {
        return false;
    }
}

bool ScriptKick::isUsingArms()
{
    if(m_current_script != NULL)
    {
        return m_data->CurrentTime <= max(m_current_script->timeFinishedWithLArm(),m_current_script->timeFinishedWithRArm()) + m_script_start_time;
    }
    else
    {
        return false;
    }
}

/*! @brief Returns true if a script uses the legs */
bool ScriptKick::isUsingLegs()
{
    if(m_current_script != NULL)
    {
        return m_data->CurrentTime <= max(m_current_script->timeFinishedWithLLeg(),m_current_script->timeFinishedWithRLeg()) + m_script_start_time;
    }
    else
    {
        return false;
    }
}

bool ScriptKick::requiresHead()
{
    if(m_current_script != NULL)
    {
        return m_current_script->usesHead();
    }
    else
    {
        return false;
    }
}

bool ScriptKick::requiresArms()
{
    if(m_current_script != NULL)
    {
        return m_script.usesLArm() or m_script.usesRArm();
    }
    else
    {
        return false;
    }
}

bool ScriptKick::requiresLegs()
{
    if(m_current_script != NULL)
    {
        return m_script.usesLLeg() or m_script.usesRLeg();
    }
    else
    {
        return false;
    }
}

void ScriptKick::doKick()
{
    if(m_data->CurrentTime <= m_current_script->timeFinished() + m_script_start_time)
    {
        // Kick has finished
        m_current_script = NULL;
        m_script_start_time = -1;
        m_kick_ready = false;
        m_kick_enabled = false;
        setArmEnabled(false, false);
        setHeadEnabled(false);
        m_kicking_leg = noLeg;
    }
    return;
}

void ScriptKick::kickToPoint(const vector<float> &position, const vector<float> &target)
{
    bool kick_begin = false
    if(kick_begin)
    {
        m_kick_ready = true;
    }
}

