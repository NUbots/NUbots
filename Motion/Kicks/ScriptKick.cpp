#include "ScriptKick.h"
#include "debug.h"
#include "debugverbositynumotion.h"
#include "Motion/Tools/MotionScript.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Motion/NUWalk.h"

ScriptKick::ScriptKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions): NUKick(walk, data, actions)
{
    m_left_kick_script = new MotionScript("KickLeft");
    m_right_kick_script = new MotionScript("KickRight");
    m_side_left_kick_script = new MotionScript("SideKickLeft");
    m_side_right_kick_script = new MotionScript("SideKickRight");
    m_current_script = NULL;
    m_script_start_time = -1;
    loadKickParameters();
}

/*! @brief Kills the kick module
 */
void ScriptKick::kill()
{
    m_current_script = NULL;
    m_script_start_time = -1;
    m_kick_ready = false;
    m_kick_enabled = false;
    setArmEnabled(false, false);
    setHeadEnabled(false);
    m_kicking_leg = noLeg;
}

void ScriptKick::stop()
{
    #if DEBUG_NUMOTION_VERBOSITY > 3
    debug << "Kick stop called. Finishing Kick." << endl;
    #endif
    if(m_script_start_time == -1 || m_current_script == NULL) {
        kill();
    }
    //stopHead();
    //stopArms();
    //stopLegs();
    //m_kick_enabled = false;
    //m_current_script = NULL;
}

ScriptKick::~ScriptKick()
{
    delete m_left_kick_script;
    delete m_right_kick_script;
    m_current_script = NULL;
}

void ScriptKick::loadKickParameters()
{
    float xMin = 5.0f;
    float xMax = 12.0f;
    //float xMax = 10.0f;
    float yMin = 3.2f;
    float yMax = 9.5f;

    m_right_kick_area = Rectangle(xMin, xMax, -yMin, -yMax);
    m_left_kick_area = Rectangle(xMin, xMax, yMin, yMax);
    m_side_right_kick_area = Rectangle(xMin, xMax, -yMin, -yMax+3.0f); //HACK: kick box less wide for side kicks
    m_side_left_kick_area = Rectangle(xMin, xMax, yMin, yMax-3.0f);
    //std::cout << "Parameters loaded." << std::endl;
    return;
}


bool ScriptKick::isActive()
{
    return m_kick_enabled;
}

bool ScriptKick::isUsingHead()
{
    if(m_current_script != NULL)
    {
        return m_data->CurrentTime > m_current_script->timeFinishedWithHead();
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
        return m_data->CurrentTime > max(m_current_script->timeFinishedWithLArm(),m_current_script->timeFinishedWithRArm());
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
        return m_data->CurrentTime > max(m_current_script->timeFinishedWithLLeg(),m_current_script->timeFinishedWithRLeg());
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
        return m_current_script->usesLArm() or m_current_script->usesRArm();
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
        return m_current_script->usesLLeg() or m_current_script->usesRLeg();
    }
    else
    {
        return false;
    }
}

void ScriptKick::doKick()
{

    if(m_current_script and m_kick_enabled and m_kick_ready and (m_script_start_time == -1))    // Check if there is a script ready, that has not been started.
    {
        if(m_walk == NULL or !m_walk->isActive())   // Either we have no walk, or we want it to be inactive.
        {
            //std::cout << "Kick Beginning." << std::endl;
            m_current_script->play(m_data, m_actions);
            m_script_start_time = m_data->CurrentTime;
        }
        
    }
    #if DEBUG_NUMOTION_VERBOSITY > 3
    debug << "Current Time: " << m_data->CurrentTime << endl;
    debug << "Walk: " << m_walk->isActive()<< endl;
    debug << "Script will finish at: " << m_current_script->timeFinished() << endl;
    #endif

    if(m_data->CurrentTime > m_current_script->timeFinished())
    {
        //std::cout << "Kick Complete. " << m_data->CurrentTime << ", " << m_current_script->timeFinished() << ", " << m_script_start_time << std::endl;
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
    bool kick_begin = false;

    if(isActive()) return;

    float ball_x = position[0];
    float ball_y = position[1];

    float target_x = target[0];
    float target_y = target[1];

    double theta = atan2(target_y - ball_y, target_x - ball_x);

    float angle_margin = mathGeneral::PI / 4.0f;

    /*if(fabs(theta) > angle_margin)
    {
        //std::cout << "Angle Too Large: " << theta << std::endl;
        return;
    }*/
    cout << theta << endl;
    // Ball is in position for left kick.
    if(m_left_kick_script->isValid() and m_left_kick_area.PointInside(ball_x, ball_y) and theta >= -angle_margin)
    {
        kick_begin = true;
        m_kicking_leg = leftLeg;
        
        
        if(theta > angle_margin and m_side_left_kick_area.PointInside(ball_x, ball_y)) {
            m_current_script = m_side_left_kick_script;
            #if DEBUG_NUMOTION_VERBOSITY > 3
            debug << "leftside kick: " << theta << endl;
            #endif
        } else if(theta <= angle_margin) {
            m_current_script = m_left_kick_script;
            #if DEBUG_NUMOTION_VERBOSITY > 3
            debug << "leftfront: " << theta << endl;
            #endif
        }
    }
    else if(m_right_kick_script->isValid() and m_right_kick_area.PointInside(ball_x, ball_y) and theta <= angle_margin)
    {
        kick_begin = true;
        m_kicking_leg = rightLeg;
        //m_current_script = m_right_kick_script;
        
        if(theta < -angle_margin and m_side_right_kick_area.PointInside(ball_x, ball_y)) {
            m_current_script = m_side_right_kick_script;
            #if DEBUG_NUMOTION_VERBOSITY > 3
            debug << "rightside: " << theta << endl;
            #endif
        } else if(theta >= angle_margin) {
            m_current_script = m_right_kick_script;
            #if DEBUG_NUMOTION_VERBOSITY > 3
            debug << "rightfront: " << theta << endl;
            #endif
        }
    }
    else
    {
        //std::cout << "No kick available for position: (" << ball_x << ", " << ball_y << ")" << std::endl;
        return;
    }

    if(kick_begin)
    {
        m_walk->stop();
        m_kick_ready = true;
        m_kick_enabled = true;
        setArmEnabled(true, true);
        setHeadEnabled(true);
        //std::cout << "Starting kick: " << toString(m_kicking_leg) << std::endl;
    }
    return;
}

