#ifndef SCRIPTKICK_H
#define SCRIPTKICK_H

#include "Motion/NUKick.h"

class ScriptKick : public NUKick
{
public:
    ScriptKick();
    ~ScriptKick();
    void doKick();
    void kickToPoint(const vector<float>& position, const vector<float>& target);

protected:

};

#endif // SCRIPTKICK_H
