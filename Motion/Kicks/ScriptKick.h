#ifndef SCRIPTKICK_H
#define SCRIPTKICK_H

#include "Motion/NUKick.h"
#include "Tools/Math/Rectangle.h"

class MotionScript;

class ScriptKick : public NUKick
{
public:
    ScriptKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~ScriptKick();
    void doKick();
    void kickToPoint(const vector<float>& position, const vector<float>& target);
    
    virtual void stop();
    virtual void kill();
    bool isActive();
    bool isUsingHead();
    bool isUsingArms();
    bool isUsingLegs();

    bool requiresHead();
    bool requiresArms();
    bool requiresLegs();

    void loadKickParameters();

protected:
    MotionScript* m_left_kick_script;
    MotionScript* m_right_kick_script;
    MotionScript* m_side_left_kick_script;
    MotionScript* m_side_right_kick_script;
    double m_script_start_time;
    MotionScript* m_current_script;

    Rectangle m_left_kick_area;
    Rectangle m_right_kick_area;
    Rectangle m_side_left_kick_area;
    Rectangle m_side_right_kick_area;

};

#endif // SCRIPTKICK_H
