#ifndef SCRIPTKICK2013_H
#define SCRIPTKICK2013_H

#include "Motion/NUKick.h"
#include "Tools/Math/Rectangle.h"

class MotionScript;

class ScriptKick2013 : public NUKick
{
public:
    ScriptKick(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions);
    ~ScriptKick();

    //! Called to tell the robot that it should begin a kick.
    //! The robot will start the first kick that it is correctly lined up for,
    //! or do nothing if no kick currently lined up.
    void kickToPoint(const std::vector<float>& position, const std::vector<float>& target);

    //! Called every frame while the robot is kicking
    void doKick();

    //! Kills the kick if has already completed (else does nothing)
    virtual void stop();
    //! Immediately stops the current kick
    virtual void kill();

    //! Returns whether the robot is currently performing a kick
    bool isActive();

    //! Returns whether the remainder of the kick script uses the head
    bool isUsingHead();
    bool isUsingArms();
    bool isUsingLegs();

    //! Returns whether this kick script uses the head at all
    bool requiresHead();
    bool requiresArms();
    bool requiresLegs();

private:
    float GetCurrentScriptTime();

    void StartKick();


protected:
    //! The currently executing script (or nullptr)
    MotionScript2013* current_script_;

    // Scripts for each of the possible kicks:
    MotionScript2013* left_kick_script_;
    MotionScript2013* right_kick_script_;
    MotionScript2013* side_left_kick_script_;
    MotionScript2013* side_right_kick_script_;

    Rectangle left_kick_area_;
    Rectangle right_kick_area_;
    Rectangle side_left_kick_area_;
    Rectangle side_right_kick_area_;
};

#endif
