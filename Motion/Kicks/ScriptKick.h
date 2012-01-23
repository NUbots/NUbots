#ifndef SCRIPTKICK_H
#define SCRIPTKICK_H

#include "Motion/NUKick.h"

class MotionScript;

class ScriptKick : public NUKick
{
public:
    ScriptKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~ScriptKick();
    void doKick();
    void kickToPoint(const vector<float>& position, const vector<float>& target);

    bool isActive();

protected:
    MotionScript* m_left_kick_script;
    MotionScript* m_right_kick_script;
    double m_script_start_time;
    MotionScript* m_current_script;

};

#endif // SCRIPTKICK_H
